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
run nonconvexExample

f = figure;
contour(g2D.xs{1},g2D.xs{2},obs,[0 0],'linewidth',2,'color','k')
axis(g2D.axis); daspect([1 1 1])
hold on
contour(g2D.xs{1},g2D.xs{2},target2D,[0 0],'linewidth',2,'color',[0 0.5 0])
gxa = plot(xa_init{1}(1),xa_init{1}(2),'+','color',[0.75 0 0]);
gxd = plot(xd_init{1}(1),xd_init{1}(2),'b*');

%---------------------------------------------------------------------------
% Voronoi line calculation
uA = compute_value(g2D,xa_init{1},velocitya,obs);
uD = compute_value(g2D,xd_init{1},velocityd,obs);

[vor, gv] = contour(g2D.xs{1},g2D.xs{2},uA-uD,[0 0],'color','b','linewidth',2);

%---------------------------------------------------------------------------
% Time to reach target set
uTa = compute_value(g2D,target2D,velocitya,obs);
uTd = compute_value(g2D,target2D,velocityd,obs);

%---------------------------------------------------------------------------
% Compute trajectories
Tf = 0.5;
dt = 0.01;

xa = xa_init{1};
xd = xd_init{1};

traja = xa;
trajd = xd;

for t = 0:dt:Tf
    % Calculate trajectories
    % Attacker takes shortest path to target
    dira = shortestPathDirection(g2D, uTa, xa);
    ua = velocitya*dira;
    
    xa = xa + ua*dt;
    traja = [traja; xa];

%     dird = optimalDirection(g2D, uTd, xd);
%     ud = velocityd*dird;

    % Defender mirrors attacker's movement
        vor_val = eval_u(uA,vor(1,2:end)',vor(2,2:end)',g2D);
    [~, mini] = min(vor_val);
    ovproj = vor(:,1+mini)';
    
    dira_perp = ovproj - xa;
    dira_perp = dira_perp / norm(dira_perp);
    dira_para = [dira_perp(2) -dira_perp(1)];
    if dira_para(2)<0, dira_para = -dira_para; end
    
    ua_perp = sum(ua.*dira_perp);
    ua_para = sum(ua.*dira_para);
    
    dird_perp = ovproj - xd;
    dird_perp = dird_perp / norm(dird_perp);
    dird_para = [dird_perp(2) -dird_perp(1)];
    if dird_para(2)<0, dird_para = -dird_para; end
    
    ud_perp = abs(ua_perp);
    ud_para = ua_para;
    
    ud = ud_perp*dird_perp + ud_para*dird_para;
    xd = xd + ud*dt;
    trajd = [trajd; xd];

    % Calculate voronoi line
    uA = compute_value(g2D,xa,velocitya,obs);
    uD = compute_value(g2D,xd,velocityd,obs);
    
    % Plot trajectories
    if exist('gxa','var'), delete(gxa); end
    if exist('gxd','var'), delete(gxd); end
    if exist('gtraja','var'), delete(gtraja); end
    if exist('gtrajd','var'), delete(gtrajd); end
    if exist('gv','var'), delete(gv); end
    
    gxa = plot(xa(1),xa(2),'+','color',[0.75 0 0]);
    gxd = plot(xd(1),xd(2),'b*');
    gtraja = plot(traja(:,1),traja(:,2),':','color',[0.75 0 0]);
    gtrajd = plot(trajd(:,1),trajd(:,2),'b:');
    
    % Plot Voronoi line
    [~, gv] = contour(g2D.xs{1},g2D.xs{2},uA-uD,[0 0],'color','b','linewidth',2);
    drawnow;
end
