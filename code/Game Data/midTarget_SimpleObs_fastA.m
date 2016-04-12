%---------------------------------------------------------------------------
% midTarget_SimpleObs_fastA
% DYNAMICS AND STATE SPACE
captureRadius = 0.1;
velocitya = 1;
velocityd = 0.9;

dims_a = [1 1 0 0];
dims_d = [0 0 1 1];

% 2D version of grid
N2D = 200;
g2D = proj2D(g, dims_a, N2D);

%---------------------------------------------------------------------------
% DOMAIN
[dom_bdry,dom_map] = std_domain(g2D);

%---------------------------------------------------------------------------
% TARGET SET
targetset.xmin = 0;
targetset.xmax = 0.2;
targetset.ymin = -0.1;
targetset.ymax = 0.1;
targetset.type = 'rect';

target4D = shapeRectangleByCorners(g,[targetset.xmin targetset.ymin -inf -inf], ...
    [targetset.xmax targetset.ymax inf inf]);
target2D = createShape2D(g2D,targetset);

%---------------------------------------------------------------------------
% OBSTACLES
% Parameters
obs1.xmin = -0.3;
obs1.xmax = -0.1;
obs1.ymin = -0.6;
obs1.ymax = 0.6;
obs1.type = 'rect';

obs2.xmin = -0.3;
obs2.xmax = 0;
obs2.ymin = 0.4;
obs2.ymax = 0.6;
obs2.type = 'rect';

obs3.xmin = obs2.xmin;
obs3.xmax = obs2.xmax;
obs3.ymin = -0.6;
obs3.ymax = -0.4;
obs3.type = 'rect';

% 4D Obstacles
[obs1_a, obs1_d] = createObstacle4D(g, obs1);
[obs2_a, obs2_d] = createObstacle4D(g, obs2);
[obs3_a, obs3_d] = createObstacle4D(g, obs3);

obs_a = shapeUnion(obs1_a, obs2_a);
obs_d = shapeUnion(obs1_d, obs2_d);
obs_a = shapeUnion(obs_a, obs3_a);
obs_d = shapeUnion(obs_d, obs3_d);

% 2D Obstacles
obs1 = createShape2D(g2D,obs1);
obs2 = createShape2D(g2D,obs2);
obs3 = createShape2D(g2D,obs3);

obs = shapeUnion(obs1, obs2);
obs2D = shapeUnion(obs, obs3);

%---------------------------------------------------------------------------
% PLAYER PARAMETERS 1
% xa_init = cell(4,1);
% xa_init{1} = [0.8 0.6];
% xa_init{2} = [0.8, 0.2];
% xa_init{3} = [0.8 -0.2];
% xa_init{4} = [0.8 -0.6];
% 
% xd_init = cell(4,1);
% xd_init{1} = [-0.7 0.7];
% xd_init{2} = [-0.7 0.2];
% xd_init{3} = [-0.7 -0.2];
% xd_init{4} = [-0.7 -0.7];
xa_init = cell(1,1);
xa_init{1} = [-0.7 0.3];

xd_init = cell(2,1);
xd_init{1} = [0.6 0.5];
xd_init{2} = [0.7 -0.3];

% ----------------------------------------------------------------------------
% 4D HJI TERMINAL AND AVOID SETS
attackerWin = target4D;

% - Defender has not captured the attacker
% Create capture condition, this is also part of the avoid set
defenderWin = (g.xs{1} - g.xs{3}) .^2 + (g.xs{2} - g.xs{4}) .^2;
defenderWin = sqrt(defenderWin) - captureRadius;

defenderWin = shapeDifference(defenderWin, attackerWin);
