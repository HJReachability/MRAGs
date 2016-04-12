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
targetset.center = 0.1;
targetset.radius = 0.1;
targetset.type = 'circ';

target4D = shapeCylinder(g, [3 4], targetset.center, targetset.radius);
target2D = createShape2D(g2D,targetset);

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

