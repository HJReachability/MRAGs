function gameFig = plot_game(game)
% Plot configurations of a game
% Game parameters are stored in a struct
% game  .n:         number of players
%       .domain:    .bdry - indices of boundary points
%                   .map -  map of domain (N by N map; -1 means outside, 1 means inside)
%       .target:    target set (N by N map; -1 means inside, 1 means outside)
%       .obstacles: obstacle map (N by N map; -1 means inside, 1 means outside)
%       .A:         position of attackers (n by 2)
%       .D:         position of defenders (n by 2)
%       .grid:      .N -    number of grid points in each dimension
%                   .L -    size of domain ([-L L]^2)
%                   .X .Y - vectors of discretized x and y values
%                   .x .y - a grid of of discretized x and y values
%                           i.e. [x,y]=ndgrid(X,Y)
%                   .infty- value of numerical infinity

% Unpack
[n, domain, target, obstacles, A, D, grid] = v2struct(game);
[N, L, X, Y, x,y, infty] = v2struct(grid);

main = figure;
xlim([-L,L]); ylim([-L,L]); axis square; box on; hold on

% domain
[~, dContour] = contour(x,y,domain.map, [0 0],'k','linewidth',3); 

% Target
[~,tContour] = contour(x,y,-target,[0 0],'b','linewidth',3);             

% Obstacles
[~, oContour] = contour(x,y,-obstacles,[0 0],'r','linewidth',3); 

% attacker position
aPlot = cell(n,1);
for i = 1:n, aPlot{i} = plot(A(i,1),A(i,2),'ro','markersize',5); end

% defender position
dPlot = cell(n,1);
for i = 1:n, dPlot{i} = plot(D(i,1),D(i,2),'bo'); end

legend([dContour tContour oContour aPlot{1} dPlot{1}], ...
    {'Domain','Target', 'Obstacle','Attacker','Defender'})

if n == 1, aPlot = aPlot{1}; dPlot = dPlot{1}; end


gameFig = v2struct(main, dContour, tContour, oContour, aPlot, dPlot);
end