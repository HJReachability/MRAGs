%---------------------------------------------------------------------------
% DYNAMICS
captureRadius = 0.1;
velocitya = 1;
velocityd = 1;

dims_a = [1 1 0 0];
dims_d = [0 0 1 1];

% 2D version of grid
g2D = proj2D(g, dims_a);

%---------------------------------------------------------------------------
% DOMAIN
[dom_bdry,dom_map] = std_domain(g2D);

%---------------------------------------------------------------------------
% TARGET SET
target.xmin = 0;
target.xmax = inf;
target.ymin = -inf;
target.ymax = inf;
target.type = 'rect';

target4D = shapeRectangleByCorners(g,[target.xmin target.ymin -inf -inf], ...
    [target.xmax target.ymax inf inf]);
target2D = createShape2D(g2D,target);

%---------------------------------------------------------------------------
% OBSTACLES
% No obstacles
obs = [];
obs_a = ones(g.N');
obs_d = ones(g.N');

%---------------------------------------------------------------------------
% PLAYER PARAMETERS 1
N = 1;
xa_init = cell(N,1);
xa_init{1} = [-0.2 0];

xd_init = cell(N,1);
xd_init{1} = [0 0];

% ----------------------------------------------------------------------------
% 4D HJI TERMINAL AND AVOID SETS
attackerWin = target4D;

% - Defender has not captured the attacker
% Create capture condition, this is also part of the avoid set
defenderWin = (g.xs{1} - g.xs{3}) .^2 + (g.xs{2} - g.xs{4}) .^2;
defenderWin = sqrt(defenderWin) - captureRadius;

attackerWin = shapeDifference(attackerWin, defenderWin);
