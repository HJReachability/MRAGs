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
% [dom_bdry,dom_map] = std_domain(g2D);
[dom_bdry,dom_map] = OLGameModified_domain(g2D);

%---------------------------------------------------------------------------
% TARGET SET
target.xmin = 0.6;
target.xmax = 0.8;
target.ymin = 0.1;
target.ymax = 0.3;
target.type = 'rect';

target4D = shapeRectangleByCorners(g,[target.xmin target.ymin -inf -inf], ...
    [target.xmax target.ymax inf inf]);
target2D = createShape2D(g2D,target);

%---------------------------------------------------------------------------
% OBSTACLES
% Parameters
obs1.xmin = -0.1;
obs1.xmax = 0.1;
obs1.ymin = -inf;
obs1.ymax = -0.3;
obs1.type = 'rect';

obs2.xmin = -0.1;
obs2.xmax = 0.1;
obs2.ymin = 0.3;
obs2.ymax = 0.6;
obs2.type = 'rect';

% obs.xmin = -0.1;
% obs.xmax = 0.1;
% obs.ymin = 0.3;
% obs.ymax = 0.6;
% obs.type = 'rect';

% 4D Obstacles
[obs1_a, obs1_d] = createObstacle4D(g, obs1);
[obs2_a, obs2_d] = createObstacle4D(g,obs2);
obs_a = shapeUnion(obs1_a, obs2_a);
obs_d = shapeUnion(obs1_d, obs2_d);
% [obs_a, obs_d] = createObstacle4D(g, obs);

% 2D Obstacles
% clrc = 0.05;
% obs1a = createShape2D(g2D,obs1,clrc);
% obs2a = createShape2D(g2D,obs2,clrc);
% obs_clrc = shapeUnion(obs1a, obs2a);
% 
obs1 = createShape2D(g2D,obs1);
obs2 = createShape2D(g2D,obs2);
obs2D = shapeUnion(obs1, obs2);
% obs2D = createShape2D(g2D,obs);
% clrc = 0.05;
% obs_clrc = createShape2D(g2D, obs2D, clrc);


%---------------------------------------------------------------------------
% PLAYER PARAMETERS
% xa_init = cell(4,1);
% xa_init{1} = [0 0.9];
% xa_init{2} = [0.8, -0.8];
% xa_init{3} = [-0.45 0.5];
% xa_init{4} = [-0.9 0.2];
% 
% xd_init = cell(4,1);
% xd_init{1} = [0.3 0.5];
% xd_init{2} = [0.3 -0.5];
% xd_init{3} = [-0.4 -0.4];
% xd_init{4} = [-0.5 -0.6];

xa_init = cell(4,1);
xa_init{1} = [-0.2 0];
xa_init{2} = [-0.5 0];
xa_init{3} = [-0.2 0.9];
xa_init{4} = [0.7 -0.9];
Na = length(xa_init);

xd_init = cell(4,1);
xd_init{1} = [0.3 0.5];
xd_init{2} = [-0.3 0.5];
xd_init{3} = [0.3 -0.5];
xd_init{4} = [-0.3 -0.5];

Nd = length(xd_init);

% ----------------------------------------------------------------------------
attackerWin = target4D;

% - Defender has not captured the attacker
% Create capture condition, this is also part of the avoid set
defenderWin = (g.xs{1} - g.xs{3}) .^2 + (g.xs{2} - g.xs{4}) .^2;
defenderWin = sqrt(defenderWin) - captureRadius;

attackerWin = shapeDifference(attackerWin, defenderWin);
attackerWin = shapeUnion(attackerWin, obs_d);
defenderWin = shapeUnion(defenderWin, obs_a);
