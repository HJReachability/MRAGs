%
%---------------------------------------------------------------------------
% Load game info and 4D HJI Data
clear all
% game = 'midTarget_LObs';
% game = 'midTarget_SimpleObs_fastA';
game = 'midTarget_LObs_fastA';
% game = 'nonconvexExample';
% game = 'OLGameModified';
load([game '_4DHJI'])
run(game)

%---------------------------------------------------------------------------
% Specify custom attacker position
xa_init = cell(1,1);
xa_init{1} = [-0.6 0.4];


% xd_init = cell(2,1);
% xd_init{1} = [0.7 0.7];
% xd_init{2} = [-0.2 0.2];
% xd_init{2} = [0.7 -0.7];

xa = xa_init{1};
% xd1 = xd_init{1};
% xd2 = xd_init{2};

xas = xa_init;
% xds = xd_init;

% Visualize
f = figure;
levels = [0 0];

[colors,hs] = visualizeGame(g2D, target2D, obs2D, xas, [], captureRadius);

% Time to reach values for attacker
ua = compute_value(g2D,xa,velocitya,obs2D,dom_map);
ut = compute_value(g2D,target2D,velocitya,obs2D,dom_map);

% Shortest path from attacker to target
path = shortestPath(ut,xa(1),xa(2),velocitya,g2D);
hp = plot(path(1,:),path(2,:),':','color',colors.attackerColor);
title('t=0')

% Distance of points to path
[pathi1, pathi2] = xy2inds(path(1,:),path(2,:),g2D);
pathi = [pathi1' pathi2'];

pathSet = ones(N2D);
for i = 1:length(pathi)
    pathSet(pathi1(i), pathi2(i)) = -1;
end
up = compute_value(g2D,pathSet,velocitya,obs2D,dom_map);

% [~, hpathSet] = contour(g2D.xs{1},g2D.xs{2},up,0:0.1:2);

% Time taken on the path
tpath = eval_u(ut,path(1,:),path(2,:),g2D);
tpath = tpath(1) - tpath;

%---------------------------------------------------------------------------
% Compute losing set for defenders
% If both defenders start in this set, they lose without being able to
% affect the shortest path from attacker to target
if 0
% Time to reach end point of shortest path
upe = compute_value(g2D,path(:,end),velocitya,obs,dom_map); 

% Set of points within tpath(end) of end point of shortest path
upe = upe - tpath(end); 
contour(g2D.xs{1},g2D.xs{2}, upe, [0 0],'color','k')

% Set of points within tpath(end)+captureRadius
upe = upe - captureRadius; 

% Set of points from which defenders will lose
AWinSet = -upe; 

contour(g2D.xs{1},g2D.xs{2}, AWinSet, [0 0],'color',colors.attackerColor)
end
% return

% ================================================
% Code below is obsolete; code above is equivalent
% ================================================
%---------------------------------------------------------------------------
% Construct maximal obstacle that cannot affect shortest path
% 2D shape
upe = compute_value(g2D,path(:,end),velocitya,obs2D,dom_map);
upe = tpath(end) - upe;
upe = shapeUnion(upe,obs2D);

[~, hmaxObs] = contour(g2D.xs{1},g2D.xs{2},upe,[0 0]);

% Downsample to 4D grid resolution
g2D_ds.dim = g2D.dim;
g2D_ds.min = g2D.min;
g2D_ds.max = g2D.max;
g2D_ds.bdry = g2D.bdry;
g2D_ds.N = g.N(1:2);
g2D_ds = processGrid(g2D_ds);
upe_ds = resampleData(g2D,upe,g2D_ds);


% upe_ds(isnan(upe_ds)) = 1;
% contour(g2D_ds.xs{1},g2D_ds.xs{2},upe_ds,[0 0]);

% Build 2D shape into 4D shape
upe_4D = repmat(upe_ds,1,g.N(3)*g.N(4));
upe_4D = reshape(upe_4D,g.N');
% upe_4D = shapeUnion(upe_4D,obs_a);

%---------------------------------------------------------------------------
% Determine which defender positions has induced obstacles that are
% contained in the maximal obstacle

% Subtract maximal obstacle from induced obstacle; we should get empty set
% if the configuration is winning for the attacker
upe_diff = shapeDifference(-data,upe_4D);

AWinSet = ones(g.N(1));
for i = 1:g.N(3)
    for j = 1:g.N(4)
        [g2, data_slice] = proj2D(g,dims_d,75,data,[g.vs{3}(i),g.vs{4}(j)]);
        
        if exist('htemp'), delete(htemp); end
        if exist('htemp2'), delete(htemp2); end
        
        [~, htemp] = contour(g2.xs{1},g2.xs{2},data_slice,[0 0],'color','b');
        htemp2 = plot(g.vs{3}(i),g.vs{4}(j),'b*');
        drawnow;
        upe_diff_ij = upe_diff(:,:,i,j);

        if all(upe_diff_ij(:) >= 0)
            htemp2 = plot(g.vs{3}(i),g.vs{4}(j),'b*');
%             msize = get(htemp2,'markersize');
%             set(htemp2,'markersize',2*msize)
            drawnow
            AWinSet(i,j) = -1;
        end
    end
end

contour(g2D_ds.xs{1},g2D_ds.xs{2}, AWinSet, [0 0])

