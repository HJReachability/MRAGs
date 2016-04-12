%---------------------------------------------------------------------------
% DYNAMICS AND STATE SPACE
captureRadius = 0.1;
velocitya = 0.8;
velocityd = 1;

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
targetset.xmin = -0.2;
targetset.xmax = 0.2;
targetset.ymin = -0.2;
targetset.ymax = 0.2;
targetset.type = 'rect';

target4D = shapeRectangleByCorners(g,[targetset.xmin targetset.ymin -inf -inf], ...
    [targetset.xmax targetset.ymax inf inf]);
target2D = createShape2D(g2D,targetset);

%---------------------------------------------------------------------------
% OBSTACLES
% Parameters
obs1.xmin = -0.5;
obs1.xmax = -0.3;
obs1.ymin = -0.3;
obs1.ymax = 0.6;
obs1.type = 'rect';

obs2.xmin = -0.5;
obs2.xmax = 0.1;
obs2.ymin = 0.4;
obs2.ymax = 0.6;
obs2.type = 'rect';

% 4D Obstacles
[obs1_a, obs1_d] = createObstacle4D(g, obs1);
[obs2_a, obs2_d] = createObstacle4D(g,obs2);

obs_a = shapeUnion(obs1_a, obs2_a);
obs_d = shapeUnion(obs1_d, obs2_d);

% 2D Obstacles
obs1 = createShape2D(g2D,obs1);
obs2 = createShape2D(g2D,obs2);
obs2D = shapeUnion(obs1, obs2);

%---------------------------------------------------------------------------
% PLAYER PARAMETERS 1
xa_init = cell(4,1);
xa_init{1} = [0.7 -0.6];
xa_init{2} = [0.7, -0.2];
xa_init{3} = [0.7 0.2];
xa_init{4} = [0.7 0.6];
Na = length(xa_init);

xd_init = cell(4,1);
xd_init{1} = [0 -0.5];
xd_init{2} = [0.2 0.6];
xd_init{3} = [0 0.7];
xd_init{4} = [-0.4 -0.5];
Nd = length(xd_init);

% ----------------------------------------------------------------------------
% 4D HJI TERMINAL AND AVOID SETS
attackerWin = target4D;

% - Defender has not captured the attacker
% Create capture condition, this is also part of the avoid set
defenderWin = (g.xs{1} - g.xs{3}) .^2 + (g.xs{2} - g.xs{4}) .^2;
defenderWin = sqrt(defenderWin) - captureRadius;

attackerWin = shapeDifference(attackerWin, defenderWin);
