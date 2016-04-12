clear all
%---------------------------------------------------------------------------
% How many grid cells?
Nx = 30;

% Create the computation grid.
g.dim = 4;
g.min = [  -1; -1; -1; -1];
g.max = [ +1; +1; +1; +1 ];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate };
% Roughly equal dx in x and y (so different N).
g.N = [ Nx; Nx; Nx; Nx ];

g = processGrid(g);

%---------------------------------------------------------------------------
% Load and visualize game
game = 'nonconvexExample';
% game = 'circleTarget_noObs';
run(game)
% Load 4D HJI Results and
% compute the costate at every grid point
load([game '_4DHJI'])
P = extractCostates(g,data); % Grid structure of costates

% Custom initial conditions
xa_init{1} = [-0.2 0.2];
xd_init{1} = [0.5 0.2];
[in, val] = in_reachset(g,data,[xa_init{1} xd_init{1}])

f = figure;
contour(g2D.xs{1},g2D.xs{2},obs,[0 0],'linewidth',2,'color','k')
axis(g2D.axis); daspect([1 1 1])
hold on
contour(g2D.xs{1},g2D.xs{2},target2D,[0 0],'linewidth',2,'color',[0 0.5 0])
gxa = plot(xa_init{1}(1),xa_init{1}(2),'+','color',[0.75 0 0]);
gxd = plot(xd_init{1}(1),xd_init{1}(2),'b*');

xd_cap = captureSet(g2D, xd_init{1},captureRadius);
[~, gxd_cap] = contour(g2D.xs{1}, g2D.xs{2}, xd_cap, [0 0], 'color','b');

%---------------------------------------------------------------------------




%---------------------------------------------------------------------------
% Voronoi line calculation
uA = compute_value(g2D,xa_init{1},velocitya,obs);
uDcap = compute_value(g2D,xd_cap,velocityd,obs);

[vor, gv] = contour(g2D.xs{1},g2D.xs{2},uA-uDcap,[0 0],'color','b','linewidth',2);

%---------------------------------------------------------------------------
% Time to reach target set
uTa = compute_value(g2D,target2D,velocitya,obs);
uTd = compute_value(g2D,target2D,velocityd,obs);

%---------------------------------------------------------------------------
% Compute trajectories
Tf = 0.8;
dt = 0.01;

xa = xa_init{1};
xd = xd_init{1};

traja = xa;
trajd = xd;

for t = 0:dt:Tf
    % Calculate trajectories
    [dira, dird] = HJIDirection(g,P,xa, xd);
    ud = velocityd*dird;
    ua = velocitya*dira;
    
%     % Attacker takes shortest path to target
%     dira = shortestPathDirection(g2D, uTa, xa);
%     ua = velocitya*dira;
%     
%     xa = xa + ua*dt;
%     traja = [traja; xa];

    % Defender mirrors attacker's movement
%     ud = mirrorVoronoiStrategy(g2D, uA, vor, xa, ua, xd);

    xa = xa + ua*dt;
    xd = xd + ud*dt;
    traja = [traja; xa];
    trajd = [trajd; xd];
    xd_cap = captureSet(g2D,xd,captureRadius);

    % Calculate voronoi line
    uA = compute_value(g2D,xa,velocitya,obs);
    uDcap = compute_value(g2D,xd_cap,velocityd,obs);
    
    % Plot trajectories
    if exist('gxa','var'), delete(gxa); end
    if exist('gxd','var'), delete(gxd); end
    if exist('gxd_cap','var'), delete(gxd_cap); end
    if exist('gtraja','var'), delete(gtraja); end
    if exist('gtrajd','var'), delete(gtrajd); end
    if exist('gv','var'), delete(gv); end
    
    gxa = plot(xa(1),xa(2),'+','color',[0.75 0 0]);
    gxd = plot(xd(1),xd(2),'b*');
    [~, gxd_cap] = contour(g2D.xs{1}, g2D.xs{2}, xd_cap, [0 0], 'color','b');
    gtraja = plot(traja(:,1),traja(:,2),':','color',[0.75 0 0]);
    gtrajd = plot(trajd(:,1),trajd(:,2),'b:');
    
    % Plot Voronoi line
    [~, gv] = contour(g2D.xs{1},g2D.xs{2},uA-uDcap,[0 0],'color','b','linewidth',2);
    drawnow;
end
