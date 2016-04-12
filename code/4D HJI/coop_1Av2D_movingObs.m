%---------------------------------------------------------------------------
% Load game info and 4D HJI Data
clear all; close all
% game = 'midTarget_LObs';
% game = 'midTarget_SimpleObs_fastA';
% game = 'midTarget_LObs_fastA';
game = 'LTarget_noObs';
% game = 'nonconvexExample';
% game = 'OLGameModified';
load([game '_4DHJI'])
run(game)

%---------------------------------------------------------------------------
% Custom initial conditions
xa_init = cell(1,1);
% xa_init{1} = [-0.6 0.3];
xa_init{1} = [-0.5 0.5];

xd_init = cell(2,1);
% xd_init{1} = [0.8 0.8];
% xd_init{2} = [-0.7 -0.7];
xd_init{1} = [0.4 0.5];
xd_init{2} = [-0.5 -0.4];

xa = xa_init{1};
xd1 = xd_init{1};
xd2 = xd_init{2};

xas = xa_init;
xds = xd_init;

% Visualize
f = figure;
levels = [0 0];

[colors,hs] = visualizeGame(g2D, target2D, obs2D, xas, xds, captureRadius);

% Time to reach values for attacker
ua = compute_value(g2D,xa,velocitya,obs2D,dom_map);

% Induced obstacles
disp('Computing induced obstacles')
tic
[g2, obs_induced1] = proj2D(g,dims_d, g2D.N(1), data, xds{1});
obs_induced1 = -obs_induced1;

[g2, obs_induced2] = proj2D(g,dims_d, g2D.N(1), data, xds{2});
obs_induced2 = -obs_induced2;

% Plot induced obstacles
[~, hoi1] = contour(g2.xs{1},g2.xs{2}, obs_induced1, levels,'color','k');
[~, hoi2] = contour(g2.xs{1},g2.xs{2}, obs_induced2, levels,'color','b');


% Compute shortest path to target given induced obstacles
obs_total = shapeUnion(obs2D,obs_induced1);
obs_total = shapeUnion(obs_total, obs_induced2);

ut_obs = compute_value(g2D,target2D,velocitya,obs_total,dom_map);

ttt_obs = eval_u(ut_obs,xa(1),xa(2),g2D);

if ttt_obs > 1000
    disp('Attacker will be captured!')

    return
end


path = shortestPath(ut_obs,xa(1),xa(2),velocitya,g2D);
toc
    
%---------------------------------------------------------------------------
% Plot shortest path

hp = plot(path(1,:),path(2,:),':','color',colors.attackerColor);
title('t=0')

drawnow;

T = 3;
dt = 0.05;

for t = dt:dt:T
    t
    tic
    %---------------------------------------------------------------------------
    % Compute shortest path to target given induced obstacles
    obs_total = shapeUnion(obs2D,obs_induced1);
    obs_total = shapeUnion(obs_total, obs_induced2);

    ut_obs = compute_value(g2D,target2D,velocitya,obs_total,dom_map);

    ASet = ua - t*velocitya; % Possible positions of A
    AList = [g2D.xs{1}(ASet<=0) g2D.xs{2}(ASet<=0)]; % List of grid points

%     ttt_obs = eval_u(ut_obs,xa(1),xa(2),g2D);
    ttt_obs = eval_u(ut_obs,AList(:,1),AList(:,2),g2D);
    [ttt_obs, ttt_obsi] = min(ttt_obs);
    
    bestA = AList(ttt_obsi,:);
    
    if ttt_obs > 1000
        disp('Attacker will be captured!')
%         delete(hp)
        delete(hmdp1)
        delete(hmdp2)
        return
    end

    path = shortestPath(ut_obs,bestA(1),bestA(2),velocitya,g2D);
    
    figure(f)
    if exist('hb','var'), delete(hb); end
    [~, hb] = contour(g2D.xs{1}, g2D.xs{2}, ASet, levels, 'color', colors.attackerColor);
    
    delete(hp)    
    hp = plot(path(1,:),path(2,:),':','color',colors.attackerColor);  
    drawnow;
%     keyboard
    %---------------------------------------------------------------------------
    % Compute shortest distance of path to induced obstacles

    % Time to induced obstacles
    uiobs1 = compute_value(g2D,obs_induced1,velocitya,obs2D,dom_map);
    uiobs2 = compute_value(g2D,obs_induced2,velocitya,obs2D,dom_map);
    
    % Time of path to induced obstacles
    pathu1 = eval_u(uiobs1,path(1,:),path(2,:),g2D);
    pathu2 = eval_u(uiobs2,path(1,:),path(2,:),g2D);
    
    % Point on path with minimum time to induced obstacles
    [minDist1, minDist1i] = min(pathu1);
    [minDist2, minDist2i] = min(pathu2);
    pPath1 = path(:,minDist1i);
    pPath2 = path(:,minDist2i);
    
    minDistPt1 = shortestPath(uiobs1,pPath1(1),pPath1(2),velocitya,g2D);
    minDistPt1 = minDistPt1(:,end)';
    minDistPt2 = shortestPath(uiobs2,pPath2(1),pPath2(2),velocitya,g2D);
    minDistPt2 = minDistPt2(:,end)';

    if exist('hmdp1','var'), delete(hmdp1); end
    if exist('hmdp2','var'), delete(hmdp2); end
    hmdp1 = plot(minDistPt1(1), minDistPt1(2), 'k.');
    hmdp2 = plot(minDistPt2(1), minDistPt2(2), 'b.');
    drawnow
    
    %---------------------------------------------------------------------------
    % Compute control input
    [~, dird1] = HJIDirection(g,P,minDistPt1,xd1);
%     [dira, dird1] = HJIDirection(g,P,xa,xd1);
    if ~dird1, [~, dird1] = HJIDirection(g,P,xa,xd1); end
    
%     [~, dird2] = HJIDirection(g,P,minDistPt2,xd2);
    [~, dird2] = HJIDirection(g,P,xa,xd2);
    if ~dird2, [~, dird2] = HJIDirection(g,P,xa,xd2); end
    
    dira = shortestPathDirection(g2D,ut_obs,xa);
    
    xa = xa + dira*velocitya*dt;
    xd1 = xd1 + dird1*velocityd*dt;
    xd2 = xd2 + dird2*velocityd*dt;
    xas{1} = xa;
    xds{1} = xd1;
    xds{2} = xd2;
    
    % Induced obstacles
    [g2, obs_induced1] = proj2D(g,dims_d, g2D.N(1), data, xds{1});
    obs_induced1 = -obs_induced1;
    
    [g2, obs_induced2] = proj2D(g,dims_d, g2D.N(1), data, xds{2});
    obs_induced2 = -obs_induced2;    
    

    figure(f)
    [colors,hs] = visualizeGame(g2D, target2D, obs2D, xas, xds, captureRadius,dom_map,hs);

    delete(hoi1)
    delete(hoi2)

    [~, hoi1] = contour(g2.xs{1},g2.xs{2}, obs_induced1, levels,'color','k');
    [~, hoi2] = contour(g2.xs{1},g2.xs{2}, obs_induced2, levels,'color','b');
    
    title(['t=' num2str(t)])
    drawnow;

    toc
%     keyboard
end