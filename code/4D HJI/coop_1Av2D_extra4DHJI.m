%
%---------------------------------------------------------------------------
% Load game info and 4D HJI Data
clear all
% game = 'midTarget_LObs';
game = 'midTarget_SimpleObs_fastA';
% game = 'midTarget_LObs_fastA';
% game = 'nonconvexExample';
% game = 'OLGameModified';
load([game '_4DHJI'])
run(game)

%---------------------------------------------------------------------------
% Custom initial conditions
xa_init = cell(1,1);
xa_init{1} = [-0.5 0];

xd_init = cell(2,1);
xd_init{1} = [0.82 0.85];
xd_init{2} = [0.9 -0.9];
% xd_init{2} = [0.7 -0.7];

xa = xa_init{1};
xd1 = xd_init{1};
xd2 = xd_init{2};

xas = xa_init;
xds = xd_init;
%---------------------------------------------------------------------------
% Compute values of each defender-attacker pair
[in1, val1] = in_reachset(g,data,[xa xd1]);
[in2, val2] = in_reachset(g,data,[xa xd2]);
disp(['val1 = ' num2str(val1)])
disp(['val2 = ' num2str(val2)])

% Make sure attacker wins against both defenders in 1 vs. 1
if ~in1
    disp('Attacker will be captured by defender 1!')
%     return;
end

if ~in2
    disp('Attacker will be captured by defender 2!')
    
%     return;
end

%---------------------------------------------------------------------------
% Visualize
figure;
visualizeGame(g2D, target2D, obs, xa_init, xd_init, captureRadius);

% levels = linspace(-0.5,0.5,50);
levels = [0 0];
[g2, slice_d1] = proj2D(g,dims_d, 200, data, xd_init{1});
contour(g2.xs{1},g2.xs{2}, slice_d1, levels,'color','k')

[g2, slice_d2] = proj2D(g,dims_d, 200, data, xd_init{2});
contour(g2.xs{1},g2.xs{2}, slice_d2, levels,'color','b')

[g2, slice_a1] = proj2D(g,dims_a, 200, data, xa_init{1});
contour(g2.xs{1},g2.xs{2}, slice_a1, levels,'color','r')
% 
% P = extractCostates(g,data);
% [ua, ud] = HJIDirection(g,P,xa_init{1},xd_init{2})
% [g2, data_slice] = proj2D(g,[0 0 1 1], 200, induced_obs, xds{1});
% contour(g2.xs{1}, g2.xs{2}, data_slice, [0 0], 'color','k')
% 
% [g2, data_slice] = proj2D(g,[1 1 0 0], 200, induced_obs, xas{1});
% contour(g2.xs{1}, g2.xs{2}, data_slice, [0 0], 'color','k')

% Create obstacle induced by defender
[g2, slice_d2_lowdim] = proj2D(g,dims_d, g.N(1), data, xd_init{2});

induced_obs = zeros(g.N');
for i = 1:g.N(3)
    for j = 1:g.N(4)
        induced_obs(:,:,i,j) = -slice_d2_lowdim;
    end
end

%---------------------------------------------------------------------------
% Rerun 4D calculation using the induced obstacle
% accuracy = 'veryHigh';
% accuracy = 'medium';
% [ dataNew, gNew, data0New ] = reachAvoid4D(accuracy, game, induced_obs, g);


%---------------------------------------------------------------------------
% View new 4D calculation
load([game '_mask.mat'])
figure;
visualizeGame(g2D, target2D, obs, xa_init, xd_init, captureRadius);

% levels = linspace(-0.5,0.5,50);
levels = [0 0];

[g2, slice_d2] = proj2D(g,dims_d, 200, dataMask, xd_init{2});
contour(g2.xs{1},g2.xs{2}, slice_d2, levels,'color','b','linewidth',2)

[g2, slice_a1] = proj2D(g,dims_a, 200, dataMask, xa_init{1});
contour(g2.xs{1},g2.xs{2}, slice_a1, levels,'color','r','linewidth',2)

% View original 4D calculation on top
[g2, slice_d2] = proj2D(g,dims_d, 200, data, xd_init{2});
contour(g2.xs{1},g2.xs{2}, slice_d2, levels,'color','b')

[g2, slice_a1] = proj2D(g,dims_a, 200, data, xa_init{1});
contour(g2.xs{1},g2.xs{2}, slice_a1, levels,'color','r')

% [g2, slice_d1] = proj2D(g,dims_d, 200, data, xd_init{1});
% contour(g2.xs{1},g2.xs{2}, slice_d1, levels,'color','k')