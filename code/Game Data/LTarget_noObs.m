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
targetset1.xmin = -0.3;
targetset1.xmax = 0.3;
targetset1.ymin = -0.3;
targetset1.ymax = -0.1;
targetset1.type = 'rect';

targetset2.xmin = 0.1;
targetset2.xmax = 0.3;
targetset2.ymin = -0.3;
targetset2.ymax = 0.5;
targetset2.type = 'rect';

target1 = createShape2D(g2D,targetset1);
target2 = createShape2D(g2D,targetset2);
target2D = shapeUnion(target1,target2);

target1 = shapeRectangleByCorners(g,[targetset1.xmin targetset1.ymin -inf -inf], ...
    [targetset1.xmax targetset1.ymax inf inf]);
target2 = shapeRectangleByCorners(g,[targetset2.xmin targetset2.ymin -inf -inf], ...
    [targetset2.xmax targetset2.ymax inf inf]);

target4D = shapeUnion(target1,target2);



%---------------------------------------------------------------------------
% OBSTACLES
% (No obstacles)
obs2D = ones(g2D.N');               % 2D obstacles
obs_a = ones(g.N');     % 4D obstacles for attacker
obs_d = ones(g.N');     % 4D obstacles for defender

%---------------------------------------------------------------------------
% PLAYER PARAMETERS
xa_init{1} = [-0.5 -0.6];
xd_init{1} = [0.8 -0.5];

%----------------------------------------------------------------------------
% TERMINAL AND AVOID SETS FOR 4D HJI COMPUTATION
attackerWin = target4D;

% - Defender has not captured the attacker
% Create capture condition, this is also part of the avoid set
defenderWin = (g.xs{1} - g.xs{3}) .^2 + (g.xs{2} - g.xs{4}) .^2;
defenderWin = sqrt(defenderWin) - captureRadius;

attackerWin = shapeDifference(attackerWin, defenderWin);

