% function [d_flag, d_region] = delay_2pgame(game)
% [d_flag, d_region] = solve_2pgame(gameName)
% Solves the two-player reach-avoid game conservatively for the defender
% using strong path defense
%
% Input:
%   gameName    - name of the game being considered; see load_game.m
%
% Outputs:
%   d_flag      - =1 if defender can win; =0 otherwise
%   d_region    - defender winning region when computation ended
%
% Mo Chen, 2013-06-28
%
clear all;
%% LOAD GAME
gameName = 'basicgame';
% [n, domain, target, obstacles, A, D, grid, speed, gameFig] = load_game(game);
[n, domain, target, obstacles, A, D, grid, speed, gameFig] = load_game(gameName);
[N,L,X,Y,x,y,infty] = v2struct(grid);
[main, dContour, tContour, oContour, aPlot, dPlot] = v2struct(gameFig);

% delete(dPlot); clear D; % If ignoring defender position
d_flag = 0;

%% ATTACKER VALUE
u_A = compute_value(grid, A, speed, obstacles, domain.map);

%% DEFENDER VALUE
u_D = compute_value(grid, D, speed, obstacles, domain.map);

%% TARGET VALUE
u_target = compute_value(grid, target, speed, obstacles, domain.map);

%% Calculate obstacle
i_dom_bdry = 110;

% ===== Find path of defense given boundary point ======
tic; [path, u_eps, i_end_pts, u_nt] = find_path(grid, target, obstacles, speed, i_dom_bdry, domain, A, u_target); toc

pilength = size(path,2);
ppathi = round(0.5*pilength);
max_ppathi = pilength;
min_ppathi = 1;

max_Avalue = -inf;
first = 1;
% for i = 1:10
while ppathi ~= max_ppathi && ppathi ~= min_ppathi
    [path_segment, path_segment_i] = path_obstacle(path, path(:,ppathi), u_A, u_D, u_nt,grid);
    
    % Convert to path segment to grid indices
    [ix, iy] = xy2inds(path_segment(1,:),path_segment(2,:),grid);
    
    % Determine which grid points are inside and outside
    [iX, iY] = ndgrid(1:N,1:N);
    path_obs_map = single(inpolygon(iX,iY,ix,iy));
    path_obs_map(path_obs_map==0)=-1;
    path_obs_map = -path_obs_map;
    
    u_target_delayed = compute_value(grid, target, speed, obstacles.*path_obs_map, domain.map);
    
    % Compute shortest path to target
    blocked_path = ComputeOptimalPath(u_target_delayed,A(1),A(2),speed,grid);
    
    % Determine path crossing relative to induced obstacle
    [path_d_ix, path_d_iy] = xy2inds(path(1,:)',path(2,:)',grid);
    path_d_i = [path_d_ix path_d_iy];
    
    [path_a_ix, path_a_iy] = xy2inds(blocked_path(1,:)',blocked_path(2,:)',grid);
    path_a_i = [path_a_ix path_a_iy];
    
    path_cross_i=find(ismember(path_d_i,path_a_i,'rows'));
    
    if exist('ppi','var'), delete(ppi); end
    ppi = plot(path(1,ppathi),path(2,ppathi),'ks');
    
   
    % Attacker value
    Avalue = eval_u(u_target_delayed,A(1),A(2),grid);
    
    % Plotting
    figure(main)
    pp = plot(path(1,:),path(2,:),'b-'); hold on     % Path of defense
    
    if Avalue > max_Avalue
        max_Avalue = Avalue; 
        longest_path = blocked_path;
        best_segment = path_segment;
    end
    if exist('bs','var'), delete(bs); end
    bs = plot(path_segment(1,:),path_segment(2,:),'k-','linewidth',3); hold on
    
    if exist('ap','var'), delete(ap); end
    ap = plot(blocked_path(1,:),blocked_path(2,:),'r-');
    

    
    title(['Attacker time to reach = ' num2str(Avalue)])
    
    pause(0.1)
    
    if first
        legendh = legend([tContour aPlot oContour pp ppi bs ap], ...
            {'target','attacker','obstacle', 'path of defense', 'entry point', 'induced obstacle','attacker path'});
        saveas(legendh,'..\fig\legend','png');
        set(legendh,'visible','off')
        pause(0.5)
    else
        pause(0.01)
    end
    
    % Save figure to png file
    gameFigName = ['..\fig\' gameName 'gameFig' num2str(ppathi) '_delay'];
    saveas(main,gameFigName,'png')
    
    first = 0;
    
    % Update search range
    if max(path_cross_i) > max(path_segment_i) % if A's path crosses "above" the obstacle
        min_ppathi = ppathi;
        ppathi = floor((ppathi + max_ppathi)/2);
    elseif min(path_cross_i) < min(path_segment_i) % if A's path crosses "below" the obstacle
        max_ppathi = ppathi;
        ppathi = ceil((ppathi + min_ppathi)/2);
    else
        keyboard
    end
    
end

if exist('bs','var'), delete(bs); end
bs = plot(best_segment(1,:),best_segment(2,:),'k-','linewidth',3); hold on

if exist('ap','var'), delete(ap); end
ap = plot(longest_path(1,:),longest_path(2,:),'r-');

title(['Attacker time to reach = ' num2str(max_Avalue)])

%
% if ~isempty(path) % if a path is found
%     % ===== Winning region for defender ======
%     tic; [d_pt, d_region, l_eps] = d_region_given_path(grid, obstacles, speed, path, u_eps{1}, u_A, domain.map); toc
%
%     if exist('d_region_union','var'),   d_region_union = d_region + d_region_union;
%     else                                d_region_union = d_region;
%     end
%     d_region_union(d_region_union>0) = infty;
%
%     % ===== Game summary figure =====
%     if exist('g1','var'), delete(g1); end
%     if exist('g2','var'), delete(g2); end
%     if exist('g3','var'), delete(g3); end
%     if exist('g4','var'), delete(g4); end
%     if exist('p','var'), delete(p); end
%     if exist('d','var'), delete(d); end
%
%     figure(main)
%     [~, g1] = contour(x,y,u_eps{1},[l_eps(1) l_eps(1)],'r');                    % First attacker winning region
%     [~, g2] = contour(x,y,u_eps{2},[l_eps(2) l_eps(2)],'r');                    % Second attacker winning region
%     [~, g3] = contour(x,y,d_region,[0 0],'g','linewidth',1.5);                            % Defender winning region for current line
%     [~, g4] = contour(x,y,d_region_union,[0 0],'g','linewidth',3);                            % Defender winning region
%
%     % plot(path(1,1),path(2,1),'b*','markersize',15);          % path endpoint
%     plot(path(1,end),path(2,end),'b.','markersize',20);      % path endpoint
%
%     p = plot(path(1,:), path(2,:),'b-', 'LineWidth',2);          % path of defense
%     d = plot(d_pt(1), d_pt(2),'gs', 'markersize',10,'markerfacecolor','g');         % best point to defend
%
%     if exist('D','var')
%         if eval_u(d_region,D(1),D(2),grid)
%             disp('Found path of defense')
%             d_flag = 1;
%             return;
%         end
%     end
%
%     legendh = legend([tContour aPlot oContour g1 g4 p d], ...
%         {'target','attacker','obstacle','attacker winning regions','defender winning region', 'path of defense', 'best point of defense'});
%     saveas(legendh,'..\fig\legend','png');
%     set(legendh,'visible','off')
%
%     % Save figure to png file
%     gameFigName = ['..\fig\' game.name 'gameFig' num2str(i_dom_bdry)];
%     saveas(main,gameFigName,'png')
%
%     first = 0;
% end
%
% d_region = d_region_union;
% % end