%---------------------------------------------------------------------------
% DYNAMICS
captureRadius = 0.1;
velocitya = 1;
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
target.xmin = -0.1;
target.xmax = 0.1;
target.ymin = -0.7;
target.ymax = -0.5;
target.type = 'rect';

target4D = shapeRectangleByCorners(g,[target.xmin target.ymin -inf -inf], ...
    [target.xmax target.ymax inf inf]);
target2D = createShape2D(g2D,target);

%---------------------------------------------------------------------------
% OBSTACLES
% Parameters
obs.xmin = -inf;
obs.xmax = 0;
obs.ymin = -0.2;
obs.ymax = 0;
obs.type = 'rect';

% 4D Obstacles
[obs_a, obs_d] = createObstacle4D(g,obs);

% 2D Obstacles
obs2D = createShape2D(g2D, obs);

%---------------------------------------------------------------------------
% PLAYER PARAMETERS
N = 1;
xa_init = cell(N,1);
xa_init{1} = [-0.2 0.2];
Na = length(xa_init);

xd_init = cell(N,1);
xd_init{1} = [0.5 0.2];
Nd = length(xd_init);

% ----------------------------------------------------------------------------
attackerWin = target4D;

% - Defender has not captured the attacker
% Create capture condition, this is also part of the avoid set
defenderWin = (g.xs{1} - g.xs{3}) .^2 + (g.xs{2} - g.xs{4}) .^2;
defenderWin = sqrt(defenderWin) - captureRadius;

attackerWin = shapeDifference(attackerWin, defenderWin);

