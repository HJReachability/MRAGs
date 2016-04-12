function [path, u_eps, i_end_pts,u_nt] = find_path(grid, target, obs, speed, i_dom_bdry, domain, A, u_target)
% [path, u_eps, i_end_pts,u_nt] = find_path(grid, target, obs, speed, i_dom_bdry, domain, A, u_target)
%  Find a shortest path with two ends of the path touching the boundary of
%  domain and the path touching the target
% 
% grid:     grid information
% target:   target set (N by N)
% speed:    speed profile (N by N)
% bdry_pt:  point from which to compute path (1 by 2)
% dom_bdry: points on the boundary (2 columns)
% dom_map:  map of domain (N by N)
% A:        attacker position
% u_target: value function of target (N by N)

checkGrid(grid);

N = grid.N(1);
X = grid.vs{1};
Y = grid.vs{2};

[dom_bdry, dom_map] = v2struct(domain);

if isempty(obs), obs = ones(N); end

toler = 0.5e-3; % tolerance for what a value of 0 means
toler2 = 3*toler;

if i_dom_bdry>1
    dom_bdry = [dom_bdry(i_dom_bdry:end,:); dom_bdry(1:i_dom_bdry-1,:)];
end

i_bdry_pt = dom_bdry(1,:);

% Check if the boundary point is inside target
if target(i_bdry_pt(1),i_bdry_pt(2))<0
    disp('Boundary point is too close to target; skipping path')
    path = [];
    u_eps = [];
    i_end_pts = [];
    return;
end

% Check if the boundary point is inside obstacle
if obs(i_bdry_pt(1),i_bdry_pt(2))<0
    disp('Boundary point is too close to obstacle; skipping path')
    path = [];
    u_eps = [];
    i_end_pts = [];
    return;
end

bdry_ptx = X(i_bdry_pt(1));
bdry_pty = X(i_bdry_pt(2));
bdry_pt = [bdry_ptx bdry_pty];
u_nt = compute_value(grid, bdry_pt, speed, obs, dom_map);       % value at boundary with no target
u_t  = compute_value(grid, bdry_pt, speed, obs.*target, dom_map);        % value at boundary with target

% difference between above two values on the boundary
diff_u = u_t - u_nt;        
dom_X = X(dom_bdry(:,1));
dom_Y = Y(dom_bdry(:,2));
diff_u_vec = eval_u(diff_u, dom_X, dom_Y, grid);
diff_u_vec(isinf(diff_u_vec)) = nan;
% Endpoint indices
i_end_pt1 = find(diff_u_vec>toler & diff_u_vec < toler2, 1,'first');
if isempty(i_end_pt1)
    disp('No suitable path endpoints found')
    path = [];
    u_eps = [];
    i_end_pts = [];
    return;
end

i_end_pt2 = find(diff_u_vec>toler & diff_u_vec < toler2, 1,'last');


end_pt1 = [X(dom_bdry(i_end_pt1,1)) Y(dom_bdry(i_end_pt1,2))];
end_pt2 = [X(dom_bdry(i_end_pt2,1)) Y(dom_bdry(i_end_pt2,2))];

% shortest path to each endpoint
path1 = shortestPath(u_t,end_pt1(1), end_pt1(2),speed,grid);
path2 = shortestPath(u_t,end_pt2(1), end_pt2(2),speed,grid);

% Convert path to indices
try
    [path1_ix, path1_iy] = xy2inds(path1(1,:)',path1(2,:)',grid);
catch
    keyboard
end
[path2_ix, path2_iy] = xy2inds(path2(1,:)',path2(2,:)',grid);
path1_i = [path1_ix path1_iy];
path2_i = [path2_ix path2_iy];

% shortest path from attacker to target
patha = shortestPath(u_target, A(1), A(2),speed,grid);
[patha_ix, patha_iy] = xy2inds(patha(1,:)',patha(2,:)',grid);
patha_i = [patha_ix patha_iy];

% check intersection of paths
path1_cross = intersect(patha_i,path1_i,'rows');
path2_cross = intersect(patha_i,path2_i,'rows');

if ~isempty(path1_cross),       path_choice = 1;
elseif ~isempty(path2_cross),   path_choice = 2;
else                            path_choice = 0;
end

switch path_choice
    case 1
        path = path1;
        end_pt = end_pt1;
        i_end_pt = [dom_bdry(i_end_pt1,1) dom_bdry(i_end_pt1,2)];
    case 2
        path = path2;
        end_pt = end_pt2;
        i_end_pt = [dom_bdry(i_end_pt2,1) dom_bdry(i_end_pt2,2)];
    otherwise
        path = [];
        u_eps = [];
        i_end_pts = [];
end

if path_choice ~= 0
    % pack output
    i_end_pts = [i_end_pt; i_bdry_pt];
    path = [path bdry_pt'];
    
    u_eps = cell(2,1);
    u_eps{1} = u_nt;
    u_eps{2} = compute_value(grid, path(:,1)', speed, obs, dom_map);       % value at boundary with no target
end


 
% % Visualization (FACTOR THIS CODE?)
% contour_lines = linspace(0, 2, 20);
% 
% figure;
% subplot(2,2,1)
% contourf(grid.xs{1},grid.xs{2},u_nt,contour_lines);
% colormap(cool); hold on;
% contour(grid.xs{1},grid.xs{2},u_nt,[0 0],'w', 'LineWidth', 3);
% axis square;
% title('End-point value with no target')
% 
% subplot(2,2,2)
% contourf(grid.xs{1},grid.xs{2},u_t,contour_lines);
% colormap(cool); hold on;
% contour(grid.xs{1},grid.xs{2},u_t,[0 0],'w', 'LineWidth', 3);
% axis square;
% 
% plot(path1(1,:), path1(2,:),'b-', 'LineWidth',3)          % path of defense
% plot(path2(1,:), path2(2,:),'b-', 'LineWidth',3)          % path of defense
% 
% title('End-point value with target as obstacle')
% 
% subplot(2,2,3:4)
% plot(diff_u_vec);
% title('End-point value along domain boundary')
% keyboard
end