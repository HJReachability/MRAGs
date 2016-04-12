function [AWinSet, uD, new_obs] = blockPath(game, xA, xD, prev_obs)
% 
% Determines alternate path given previous obstacles. 
%
% Inputs:
%
% Outputs:
%
% Mo Chen, 2014-04-15

%% Initialization
load([game '_4DHJI'])
run(game);

xD0 = xD;

tMax = 5;
dt = 0.025;
t = dt:dt:tMax;

if ~iscell(prev_obs)
    prev_obs = {prev_obs};
end

% Extract induced obstacle
% data is negative where defender loses, including inside obs_d
% data is positive where defender wins, including inside obs_a
% [~, obs_induced] = proj2D(g, dims_d, N2D, shapeDifference(data, obs_a), xD); 
[~, obs_induced] = proj2D(g, dims_d, N2D, data, xD); 
obs_induced = -obs_induced;
obs_induced = shapeDifference(obs_induced, obs2D- 0.01);

new_obs = cell(length(t),1);
new_obs{1} = shapeUnion(prev_obs{1}, obs_induced);
total_obs = shapeUnion(prev_obs{end}, obs_induced);

% Defender control signal
uD = zeros(length(t),2);

% Attacker's time to reach target function
uA = compute_value(g2D, xA, velocitya, prev_obs{1}, dom_map);

visualizeGame(g2D, target2D, prev_obs{1}, {xA}, {xD}, captureRadius, dom_map);

for i = 1:length(t)
    %% Calculate new path to target taking into account of new obstacle
    ut = compute_value(g2D, target2D, velocitya, total_obs, dom_map);
    
    % Possible positions of attacker
    aposi = max([i length(prev_obs)]);
    APosSet = uA - t(aposi); 
    
    APosList = [g2D.xs{1}(APosSet<=0) g2D.xs{2}(APosSet<=0)];
    APosSetValues = eval_u(g2D, ut, APosList);

    % Path from closest point to the target given new obstacle
    [~, mini] = min(APosSetValues);
    
    P2D = extractCostates(g2D,ut);   % Gradient of value function
%     keyboard
    path = shortestPathP(g2D, P2D, ut, APosList(mini,:), velocitya);
    
%     path = shortestPath(ut, APosList(mini,1), APosList(mini,2), velocitya, g2D);
    
    if size(path,2) <2
        disp('Attacker will be captured!')
        new_obs(i+1:end) = [];
        uD(i:end,:) = [];
        return
    end
    
    %% Determine attacker winning set
    AWinSet = interceptSet(g2D, obs2D, dom_map, uA, path, velocityd, captureRadius);
    contour(g2D.xs{1}, g2D.xs{2}, AWinSet, [0 0], 'k');    
    
    %% Terminate if defender is inside attacker winning set
    vd = eval_u(g2D, AWinSet, xD0);
    plot(xD0(1), xD0(2), 'b*', 'markersize',3)
    drawnow;
    
    if vd<=0 && i>length(prev_obs)
        disp('Defender cannot intercept attacker!')
        
        new_obs(i+1:end) = [];
        uD(i:end,:) = [];    
        
        return
    end

    %% Determine best point on the path to go for
    % Compute time it takes induced obstacle to reach each point on path
    disp('Attempting to intercept')
    u_inducedObs = compute_value(g2D, obs_induced, velocityd, obs2D, dom_map);
    t_path2Obs = eval_u(g2D, u_inducedObs, path');
    
    % Compute time it takes attacker to get to each point on the path
    ut = compute_value(g2D, target2D, velocitya, new_obs{i}, dom_map);
    
    tpath = eval_u(g2D, ut, path');
    tpath = tpath(1) - tpath;
    
    % Compute the difference in time between obstacle to path and initial
    % position to path
    t_diff = tpath - t_path2Obs;
    
    % Compute maximum of such times
    [~, t_diff_max_i] = max(t_diff);
    path_x = path(:,t_diff_max_i);
    
    %% Determine which point of the induced obstacle to push forward
    % Compute point on the induced obstacle that's closest to the above point
    % on path
    u_pep = compute_value(g2D, path_x, velocityd, obs2D, dom_map);
    obs_induced_list = [g2D.xs{1}(obs_induced<=0 & obs2D>0) g2D.xs{2}(obs_induced<=0 & obs2D>0)];
    obs_induced_v = eval_u(g2D, u_pep, obs_induced_list);
    [~, obs_induced_i] = min(obs_induced_v);
    
    obs_induced_x = obs_induced_list(obs_induced_i, :);   
    
    %% Update player position
	[~, dird] = HJIDirection(g, P, obs_induced_x, xD);
    xD = xD + velocityd*dird*dt;
    uD(i,:) = dird;
    
    dird
%     keyboard
    
    %% Calculate new induced obstacle for next time step
    [~, obs_induced] = proj2D(g, dims_d, N2D, data, xD);
    obs_induced = -obs_induced;
    obs_induced = shapeDifference(obs_induced, obs2D - 0.01);
    % Obstacle indices
    obsi = min(i+1, length(prev_obs));
    new_obs{i+1} = shapeUnion(prev_obs{obsi}, obs_induced);
    total_obs = shapeUnion(prev_obs{end}, obs_induced);
    
    
    hold off
    visualizeGame(g2D, target2D, prev_obs{obsi}, {xA}, {xD}, captureRadius, dom_map);
    contour(g2D.xs{1}, g2D.xs{2}, APosSet, [0 0], 'r')
    contour(g2D.xs{1}, g2D.xs{2}, obs_induced, [0 0], 'b')  
    plot(obs_induced_x(1), obs_induced_x(2), 'b*')

    
    plot(path(1,:), path(2,:), 'r:')
    
    
end
end