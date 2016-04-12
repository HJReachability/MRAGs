clear all;

% Analysis of run time for calculating winning regions given path
load('OLGameModified_dregions_Rc_all.mat')
load('OLGameModified_times.mat')

% GRID
Nx = 45;

% Create the computation grid.
g.dim = 4;
g.min = [  -1; -1; -1; -1];
g.max = [ +1; +1; +1; +1 ];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate };
% Roughly equal dx in x and y (so different N).
g.N = [ Nx; Nx; Nx; Nx ];

g = processGrid(g);

run('OLGameModified')

% Compute area of finest boundary discretization
npath = npaths{end};
d_regions = eval(['d_regions' npath]);

areasFull = zeros(length(d_regions),1);

for i = 1:length(d_regions)
    areasFull(i) = nnz(d_regions{i}<=0);
end

% Compute area of coarser boundary discretization
d_regions = cell(length(npaths),1);
areasPartialFrac = zeros(length(npaths),1);
compTimes = zeros(length(npaths),1);
for j = 1:length(npaths)
    d_regions{j} = eval(['d_regions' npaths{j}]);
    areasPartial = zeros(length(d_regions{j}),1);
    
    for i = 1:length(d_regions{j})
        areasPartial(i) = nnz(d_regions{j}{i}<=0);
    end

    areasPartialFrac(j) = mean(areasPartial./areasFull);
    compTimes(j) = eval(['avg_solve_2p_time' npaths{j}]);
end


% Aggregrate path computation times
paths_time = zeros(length(npaths),1);

for i = 1:length(npaths)
    paths_set_time = eval(['paths_set_time' npaths{i}]);
    paths_u_time = eval(['paths_u_time' npaths{i}]);
    paths_time(i) = paths_set_time + paths_u_time;
end


% Plot
npaths_num = zeros(length(npaths),1);
for j = 1:length(npaths), npaths_num(j) = str2num(npaths{j}); end

figure;


subplot(3,1,1)
plot(npaths_num, areasPartialFrac, 'bo-')
ylabel('2D Slice Area Frac.')

subplot(3,1,2)
plot(npaths_num, paths_time, 'kx-')
ylabel('Paths comp. time (s)')

subplot(3,1,3)
plot(npaths_num, compTimes, 'r.-')
ylabel('2D slice comp. time (s)')

subplot(3,1,1),title('Algorithm Performance vs. Number of Boundary Points')
subplot(3,1,3),xlabel('Number of boundary points')

