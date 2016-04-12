%---------------------------------------------------------------------------
% Load game info and 4D HJI Data
clear all
% game = 'midTarget_LObs';
% game = 'midTarget_SimpleObs_fastA';
% game = 'midTarget_LObs_fastA';
% game = 'nonconvexExample';
game = 'OLGameModified';
load([game '_4DHJI'])
run(game)

%---------------------------------------------------------------------------
% Custom initial conditions
xa_init = cell(1,1);
xa_init{1} = [-0.5 0];

xd_init = cell(1,1);
xd_init{1} = [0.9 -0.9];
% xd_init{2} = [0.7 -0.7];

xa = xa_init{1};
xd = xd_init{1};

xas = xa_init;
xds = xd_init;
%---------------------------------------------------------------------------


%---------------------------------------------------------------------------
% Integrate trajectory
P = extractCostates(g,data);
dt = 0.025;
Tf = 1.5;


f1 = figure;
f2 = figure;
for t = 0:dt:Tf
    [dira, dird] = HJIDirection(g,P,xa,xd);
    
    if norm(xa-xd) <= captureRadius
        disp('Attacker has been captured!')
        return;
    end
    
%     if rand<=0.5
%         dira = rand(1,2);
%         dira = dira/norm(dira);
%     end
%     
%     dira = [0 -1];
%     
    xa = xa + dira*velocitya*dt;
    xd = xd + dird*velocityd*dt;
    
    xas{1} = xa;
    xds{1} = xd;
    
    plotReachSets(g,g2D,target2D,obs2D,data,...
        xas,xds,captureRadius,f1,f2, dims_a, dims_d, dom_map)
    
    % Plot induced obstacle
    
end
