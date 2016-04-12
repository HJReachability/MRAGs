%---------------------------------------------------------------------------
% Load game info and 4D HJI Data
clear all
% game = 'midTarget_LObs';
% game = 'nonconvexExample';
game = 'OLGameModified';
load([game '_4DHJI'])
run(game)

% Custom initial conditions
% xa_init{1} = [0.017 -0.28];
% xd_init{1} = [-0.3455 -0.3828];
% xd_init{2} = [0.25 -0.3];

% xa_init{1} = [-0.4 0.5];
% xd_init{1} = [-0.65 0.4];
% xd_init{2} = [0.7 0.7];

xa_init = cell(1,1);
xa_init{1} = [-0.3 0.55];

xd_init = cell(2,1);
xd_init{1} = [-0.3 0.9];
xd_init{2} = [-0.5 -0.3];
%---------------------------------------------------------------------------
% Visualize game
f = figure;
contour(g2D.xs{1},g2D.xs{2},obs,[0 0],'linewidth',2,'color','k')
axis(g2D.axis); daspect([1 1 1])
hold on
contour(g2D.xs{1},g2D.xs{2},target2D,[0 0],'linewidth',2,'color',[0 0.5 0])

gxa = plot(xa_init{1}(1),xa_init{1}(2),'+','color',[0.75 0 0]);
gxd1 = plot(xd_init{1}(1),xd_init{1}(2),'b.');
gxd2 = plot(xd_init{2}(1),xd_init{2}(2),'b*');

xd1_cap = captureSet(g2D, xd_init{1},captureRadius);
[~, gxd1_cap] = contour(g2D.xs{1}, g2D.xs{2}, xd1_cap, [0 0], 'color','b');

xd2_cap = captureSet(g2D, xd_init{2},captureRadius);
[~, gxd2_cap] = contour(g2D.xs{1}, g2D.xs{2}, xd2_cap, [0 0], 'color','b');
drawnow;

f2 = figure;
%---------------------------------------------------------------------------
P = extractCostates(g,data); % Grid structure of costates
small = 5e-3;

dt = 0.025;
Tf = 1.5;

xa = xa_init{1};
xd1 = xd_init{1};
xd2 = xd_init{2};

traja = xa;
trajd1 = xd1;
trajd2 = xd2;

for t = 0:dt:Tf
    % Defenders just play optimally against the single attacker
    [~, dird1] = HJIDirection(g,P,xa, xd1);
    [~, dird2] = HJIDirection(g,P,xa, xd2);

%     [~, dird1] = HJIDirection2(g,data,xa, xd1);
%     [~, dird2] = HJIDirection2(g,data,xa, xd2);
    
    if dird1 == [0 0], disp('Defender 1 stopped!'); end
    if dird2 == [0 0], disp('Defender 2 stopped!'); end
    
    if min(norm(xa-xd1), norm(xa-xd2))<= captureRadius
        disp('Attacker has been captured!')
        return;
    end
    
    % Evaluate value function to determine which defender is furthest away
    % from reachable set
    [in1, val1] = in_reachset(g,data,[xa xd1]);
    [in2, val2] = in_reachset(g,data,[xa xd2]);
    
    if ~in1
        disp('Attacker will be captured by defender 1!')
        return;
    end
    
    if ~in2
        disp('Attacker will be captured by defender 2!')
        return;
    end
    
    if eval_u(target2D, xa(1),xa(2), g2D) <= 0
        disp('Attacker is inside target!')
        return;
    end
%     dira1 = HJIDirection(g,P,xa,xd1);
%     dira2 = HJIDirection(g,P,xa,xd2);
    
    dira1 = HJIDirection2(g,data,xa,xd1);
    dira2 = HJIDirection2(g,data,xa,xd2);

    if      val1 >= -small, dira = dira1;
    elseif  val2 >= -small, dira = dira2;
    else
        dira = dira1/abs(val1) + dira2/abs(val2);
        dira = dira/norm(dira);
    end
    
%     if abs(val1 - val2) < small % Pick control "half way between" both defenders
%         dira1 = HJIDirection(g,P,xa,xd1);
%         dira2 = HJIDirection(g,P,xa,xd2);
%         dira = dira1 + dira2;
%         dira = dira/norm(dira);
%     elseif val1 > val2          % Pick control against defender 1
%         dira = HJIDirection(g,P,xa,xd1);
%     else                        % Pick control against defender 2
%         dira = HJIDirection(g,P,xa,xd2);
%     end

    % Update state
    xa = xa + dira*velocitya*dt;
    xd1 = xd1 + dird1*velocityd*dt;
    xd2 = xd2 + dird2*velocityd*dt;
    xd1_cap = captureSet(g2D, xd1, captureRadius);
    xd2_cap = captureSet(g2D, xd2, captureRadius);
    
    traja = [traja; xa];
    trajd1 = [trajd1; xd1];
    trajd2 = [trajd2; xd2];
    
    % Visualize
    % Plot trajectories
    figure(f)
    if exist('gxa','var'), delete(gxa); end
    if exist('gxd1','var'), delete(gxd1); end
    if exist('gxd2','var'), delete(gxd2); end
    if exist('gxd1_cap','var'), delete(gxd1_cap); end
    if exist('gxd2_cap','var'), delete(gxd2_cap); end
    if exist('gtraja','var'), delete(gtraja); end
    if exist('gtrajd1','var'), delete(gtrajd1); end
    if exist('gtrajd2','var'), delete(gtrajd2); end
        
    gxa = plot(xa(1),xa(2),'+','color',[0.75 0 0]);
    gxd1 = plot(xd1(1),xd1(2),'b.');
    gxd2 = plot(xd2(1),xd2(2),'b*');
    
    [~, gxd1_cap] = contour(g2D.xs{1}, g2D.xs{2}, xd1_cap, [0 0], 'color','b');
    [~, gxd2_cap] = contour(g2D.xs{1}, g2D.xs{2}, xd2_cap, [0 0], 'color','b');
    
    gtraja = plot(traja(:,1),traja(:,2),':','color',[0.75 0 0]);
    gtrajd1 = plot(trajd1(:,1),trajd1(:,2),'b:');
    gtrajd2 = plot(trajd2(:,1),trajd2(:,2),'b:');
    
    drawnow;
    
    figure(f2)
    plot(t,val1,'b.')
    hold on

    plot(t,val2,'b*')
    xlim([0 Tf])
    ymm = ylim;
    ylim([ymm(1) 0])    
    
    drawnow;
    directory = [game '_fig1'];
    file_f = [directory '/1v2_' num2str(t*1000)];
    figure(f)
    export_fig(file_f, '-png', '-transparent');

    file_f2 = [directory '/1v2_vals' num2str(t*100)];
    figure(f2)
    export_fig(file_f2, '-png', '-transparent');
end