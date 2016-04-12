
%---------------------------------------------------------------------------
% Load game info and 4D HJI Data
clear all
% game = 'midTarget_LObs';
game = 'midTarget_SimpleObs_fastD';
% game = 'midTarget_LObs_fastA';
% game = 'nonconvexExample';
% game = 'OLGameModified';
load([game '_4DHJI'])

run(game)

%---------------------------------------------------------------------------
% Custom initial conditions
xa_init = cell(2,1);
xa_init{1} = [-0.6 0.2];
xa_init{2} = [-0.6 -0.2];

xd_init = cell(1,1);
xd_init{1} = [0.5 0];

xa1 = xa_init{1};
xa2 = xa_init{2};

xd = xd_init{1};
xdcap = captureSet(g2D,xd,captureRadius);

xas = xa_init;
xds = xd_init;

xatraj = cell(2,1);
xdtraj = cell(1,1);

xatraj{1} = xa1;
xatraj{2} = xa2;
xdtraj{1} = xd;

hxatraj = cell(2,1);
hxdtraj = cell(1,1);

%---------------------------------------------------------------------------
% Compute time to reach values of the players
ua1 = compute_value(g2D,xa1,velocitya,obs,dom_map);
ua2 = compute_value(g2D,xa2,velocitya,obs,dom_map);
ud = compute_value(g2D,xdcap,velocityd,obs,dom_map);

capLocSet = ua1-ud;
%---------------------------------------------------------------------------
% Visualize
figure;
[colors, hs] = visualizeGame(g2D, target2D, obs, xa_init, xd_init, captureRadius);

contour(g2D.xs{1},g2D.xs{2},capLocSet,[0 0], 'color','b')

%---------------------------------------------------------------------------
% Create all possible A2-D configurations after capture
capLocList = [g2D.xs{1}(capLocSet<=0) g2D.xs{2}(capLocSet<=0)];
capTimeList = eval_u(ud,capLocList(:,1),capLocList(:,1),g2D);

A2PossibleWinSets = cell(size(capLocList,1),1);
% For every possible capture location (D captures A1
for i = 1:size(capLocList,1)
    % Determine locations for A2 to win
    [g2, A2WinSet] = proj2D(g,dims_d,N2D,data,capLocList(i,:));
    
    % Compute capture time
    capTime = capTimeList(i);    
    
    % Possible locations for A2 to be at at capture time
    A2LocSet = ua2 - capTime;
    
    % Find intersection between A2's winning location and A2's
    % possible location
    A2PossibleWinSets{i} = shapeIntersection(A2LocSet, A2WinSet);
% 
%     if exist('capLocPlot','var'), delete(capLocPlot); end
%     if exist('A2WinPlot','var'), delete(A2WinPlot); end
%     if exist('A2LocPlot','var'), delete(A2LocPlot); end    
    
    if any(A2PossibleWinSets{i}(:)<=0)
        disp('Attackers win!')
    
        capLocPlot = plot(capLocList(i,1),capLocList(i,2),'b.');
        [~, A2WinPlot] = contour(g2D.xs{1},g2D.xs{2},A2WinSet,[0 0],'color','r');
        [~, A2LocPlot] = contour(g2D.xs{1},g2D.xs{2},A2LocSet,[0 0],'color','k');
        drawnow;
        
        capLoc = capLocList(i,:);
        break;
    end
end

% Compute time to reach capture location
ucap = compute_value(g2D,capLoc,velocitya,obs,dom_map);

% Compute time to reach A2 winning location
ind = find(A2PossibleWinSets{i} <= 0, 1, 'first');
A2WinLoc = [g2D.xs{1}(ind) g2D.xs{2}(ind)];
uA2Win = compute_value(g2D,A2WinLoc,velocitya,obs,dom_map);

% Extract costates for 4D HJI optimal control
P = extractCostates(g,data);

T = 3;
dt = 0.025;

captured = false;
for t = 0:dt:T
    % Capture attacker 1 if he hasn't been captured already
    % Otherwise, go for attacker 2
    
    if norm(xa1 - xd) <= captureRadius, captured = true; end
    if ~captured
        % Compute optimal direction
        dira1 = shortestPathDirection(g2D,ucap,xa1);
        dira2 = shortestPathDirection(g2D,uA2Win,xa2);
        dird = shortestPathDirection(g2D,ucap,xd);
        
        xa1 = xa1 + dira1*velocitya*dt;
        
    else
        [dira2, dird] = HJIDirection(g,P,xa2,xd);
    end
    
    xa2 = xa2 + dira2*velocitya*dt;    
    xd = xd + dird*velocityd*dt;
    xdcap = captureSet(g2D,xd,captureRadius);
    
    xas{1} = xa1;
    xas{2} = xa2;
    xds{1} = xd;
    
    xatraj{1} = [xatraj{1}; xa1];
    xatraj{2} = [xatraj{2}; xa2];
    xdtraj{1} = [xdtraj{1}; xd];
    
    % Update figure
    delete(hs.hxas{1});
    delete(hs.hxas{2});
    delete(hs.hxds{1});
    delete(hs.hxdcaps{1});
    delete(hs.ht);
    delete(hs.ho);
    
    if ~isempty(hxatraj{1}), delete(hxatraj{1}); end
    if ~isempty(hxatraj{2}), delete(hxatraj{2}); end
    if ~isempty(hxdtraj{1}), delete(hxdtraj{1}); end
    
    % Plot
    [colors, hs] = visualizeGame(g2D,target2D,obs,xas,xds,captureRadius);
%     hs.hxas{1} = plot(xa1(1),xa1(2),'+','color',colors.attackerColor);
%     hs.hxas{2} = plot(xa2(1),xa2(2),'+','color',colors.attackerColor);
%     hs.hxds{1} = plot(xd(1), xd(2), '*','color',colors.defenderColor);
%     [~, hs.hxdcaps{1}] = contour(g2D.xs{1}, g2D.xs{2}, xdcap, [0 0], ...
%         'color',colors.defenderColor);

    hxatraj{1} = plot(xatraj{1}(:,1), xatraj{1}(:,2), ':', 'color', colors.attackerColor);
    hxatraj{2} = plot(xatraj{2}(:,1), xatraj{2}(:,2), ':', 'color', colors.attackerColor);
    hxdtraj{1} = plot(xdtraj{1}(:,1), xdtraj{1}(:,2), ':', 'color', colors.defenderColor);
    title(['t=' num2str(t)])
    drawnow;

end