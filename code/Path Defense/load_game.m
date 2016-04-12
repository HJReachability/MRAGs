function [n, domain, target, obstacles, A, D, grid, speed, gameFig, game] = load_game(gameName)
% [n, domain, target, obstacles, A, D, grid, speed, gameFig, game] = load_game(gameName)
% Load game file and unpack the game variables
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

if ischar(gameName)
    load('games',gameName);
    game = eval(gameName);
else
    game = gameName;
end
% Plot
gameFig = plot_game(game);

[n, domain, target, obstacles, A, D, grid, speed] = v2struct(game);

end