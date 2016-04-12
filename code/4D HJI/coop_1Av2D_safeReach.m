%
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
% Specify custom player positions
xa_init = cell(1,1);
% xa_init{1} = [-0.6 0.4];
xa_init{1} = [-0.5 0.5];

% xd_init = cell(2,1);
% xd_init{1} = [0.8 0.9];
% xd_init{2} = [-0.9 -0.8];
xd_init = cell(1,1);
% xd_init{1} = [-0.9 -0.8];
xd_init{1} = [0.6 0.5];

xa = xa_init{1};
% xd1 = xd_init{1};
% xd2 = xd_init{2};
xd = xd_init{1};

xas = xa_init;
xds = xd_init;

tic

% Visualize
f = figure;
levelsPlot = [0 0];
[colors,hs] = visualizeGame(g2D, target2D, obs2D, xas, xds, captureRadius);

T = 1;
dt = 0.025;

level_num = 0;
levelSets = cell(length(0:dt:T),1);
levels = zeros(length(0:dt:T),1);

for t = 0:dt:T
    % Induced obstacles
    total_obs = obs2D;
    [g2, obs_induced] = proj2D(g,dims_d, N2D, data, xd);
    obs_induced = -obs_induced;
    obs_induced = shapeDifference(obs_induced,obs2D);
    total_obs = shapeUnion(total_obs, obs_induced);

    % Save obstacle level set and level
    level_num = level_num + 1;
    levelSets{level_num} = obs_induced;
    levels(level_num) = t;
    
    % Time to reach attacker function
    uA = compute_value(g2D,xa,velocitya,obs2D,dom_map);
    
    % Shortest path from attacker to target
    ut = compute_value(g2D,target2D,velocitya,total_obs,dom_map);
    if t == 0
        path = shortestPath(ut,xa(1),xa(2),velocitya,g2D);
    else
        APosSet = uA - t; % Possible positions of attacker
        if exist('hAPosSet','var'), delete(hAPosSet); end
        [~, hAPosSet] = contour(g2D.xs{1},g2D.xs{2},APosSet,levelsPlot,'color',colors.attackerColor);
        
        APosList = [g2D.xs{1}(APosSet<=0) g2D.xs{2}(APosSet<=0)];
        APosSetValues = eval_u(ut,APosList(:,1),APosList(:,2),g2D);
        [~, mini] = min(APosSetValues);
        path = shortestPath(ut,APosList(mini,1),APosList(mini,2),velocitya,g2D);
    end
    
    % Check if attacker has been captured for sure
    if size(path,2)<2
%         delete(ho);
%         delete(hx);
%         delete(hp);
        
%         if ~isempty(hxd_obs{i}), delete(hxd_obs{i}); end
        [~, hxd_obs] = contour(g2D.xs{1},g2D.xs{2},obs_induced,...
            levelsPlot,'color',colors.defenderColor);
        
        [colors,hs] = visualizeGame(g2D,target2D,obs2D,xas,xds,captureRadius,dom_map,hs);
        return
    end
    
    if exist('hp','var'), delete(hp); end
    hp = plot(path(1,:),path(2,:),':','color',colors.attackerColor);
    title(['t=' num2str(t)])
    
    % Distance of points to path
    [pathi1, pathi2] = xy2inds(path(1,:),path(2,:),g2D);
    pathi = [pathi1' pathi2'];
    
    pathSet = ones(N2D);
    for i = 1:length(pathi)
        pathSet(pathi1(i), pathi2(i)) = -1;
    end
    up = compute_value(g2D,pathSet,velocitya,obs2D,dom_map);
    
    % [~, hpathSet] = contour(g2D.xs{1},g2D.xs{2},up,0:0.1:2);
    
    % Time taken on the path
    tpath = eval_u(ut,path(1,:),path(2,:),g2D);
    tpath = tpath(1) - tpath;
    
    %---------------------------------------------------------------------------
    % Compute losing set for defenders
    % If both defenders start in this set, they lose without being able to
    % affect the shortest path from attacker to target
   
    % Set of points from which defenders will lose
    AWinSet = interceptSet(g2D, obs2D, dom_map, uA, path, velocityd, captureRadius);
    
    if exist('hAWinSet','var'), delete(hAWinSet); end
    [~, hAWinSet] = contour(g2D.xs{1},g2D.xs{2}, AWinSet, [0 0],'color',colors.attackerColor);
    
    % If defenders have no hope of winning, end
    vd = eval_u(AWinSet,xd_init{1}(1), xd_init{1}(2), g2D);
    if vd<=0
        disp('Defender cannot intercept attacker!')
%         delete(ho);
%         delete(hx);
%         delete(hp);
        
        if exist('hxd_obs','var'), delete(hxd_obs); end
        [~, hxd_obs] = contour(g2D.xs{1},g2D.xs{2},obs_induced,...
            levelsPlot,'color',colors.defenderColor);
        
        [colors,hs] = visualizeGame(g2D,target2D,obs2D,xas,xds,captureRadius,dom_map,hs);       

        
        return
    end
    toc
    
 
    %---------------------------------------------------------------------------
    % Compute the best point on the shortest path to try to cut off
    
    % Extract induced obstacle
        if exist('hxd_obs','var'), delete(hxd_obs); end
        [~, hxd_obs] = contour(g2D.xs{1},g2D.xs{2},obs_induced, levelsPlot,'color',colors.defenderColor);
    
    % Compute distance from path to induced obstacle
    u_inducedObs = compute_value(g2D,obs_induced,velocityd,obs2D,dom_map);
%     contour(g2D.xs{1},g2D.xs{2},u_inducedObs,linspace(0,2,20))
    t_path2Obs = eval_u(u_inducedObs,path(1,:),path(2,:),g2D);
    
    % Compute the difference in time between obstacle to path and initial
    % position to path
    t_diff = tpath - t_path2Obs;
    
    % Compute maximum of such times
    [t_diff_max, t_diff_max_i] = max(t_diff);
    
    if exist('ho','var'), delete(ho); end
    ho = plot(path(1,t_diff_max_i),path(2,t_diff_max_i),'bo');
    
    % Compute point on the induced obstacle that's closest to the above point
    % on path
    u_pep = compute_value(g2D,path(:,t_diff_max_i),velocitya,obs2D,dom_map);
    obs_induced_list = [g.xs{1}(obs_induced<=0) g.xs{2}(obs_induced<=0)];
    obs_induced_v = eval_u(u_pep, obs_induced_list(:,1), obs_induced_list(:,2), g2D);
    [~, obs_induced_i] = min(obs_induced_v);
    obs_induced_x = obs_induced_list(obs_induced_i, :);
    
    if exist('hx','var'), delete(hx); end
    hx = plot(obs_induced_x(1),obs_induced_x(2),'kx');
    [colors,hs] = visualizeGame(g2D,target2D,obs2D,xas,xds,captureRadius,dom_map,hs);
    drawnow;
    
    % Update defender positions
    [~, dird] = HJIDirection(g,P,obs_induced_x,xd);
    
    xd = xd + velocityd*dird*dt;
    xds{1} = xd;
end

levelSets(level_num+1:end) = [];
levels(level_num+1:end) = [];