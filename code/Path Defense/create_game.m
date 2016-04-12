function game = create_game(n, domain, target, obstacles, A, D, grid, speed)
% Create a new game configuration
% Game parameters are stored in a struct
%
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
%
% Mo Chen, 2013-06-07
%

%% GAME INFO
if nargin<7
    % Grid
    N = 200;
    L = 1;
    X = linspace(-L,L,N);
    Y = linspace(-L,L,N);
    infty = 1e6;
    [x,y] = ndgrid(X,Y);
    
    grid = v2struct(N,L,X,Y,x,y,infty);
end

if nargin<1, n = 1; end             % # of attackers (only works with 1 attacker right now)

if nargin<2, 
    [dom_bdry,dom_map] = std_domain(grid); 
    domain = v2struct(dom_bdry, dom_map, {'fieldnames','bdry','map'});
end 

if nargin<3
    target = sqrt( (x+0.2).^2 + (y-L).^2 ) - 0.1;
    target(target<=0) = -1;
    target(target>0) = 1;
end

if nargin<4, obstacles = ones(N,N); end          % Draw obstacles? How many?
if nargin<5, A = [-0.6 0]; end      % Draw attacker position?
if nargin<6, D = [0.6 0]; end      % Specify defender position?


[N,L,X,Y,x,y,infty] = v2struct(grid);
if nargin<7, speed = ones(N,N); end   % Speed (equal, uniform speeds right now)

% ===== DOMAIN =====
fig_draw = figure; hold on
if ~isstruct(domain)
    disp('Draw domain'); 
    [dom_bdry, dom_map] = draw_set(grid); 
    domain = v2struct(dom_bdry, dom_map, {'fieldnames','bdry','map'});
end

% ===== TARGET SET =====
if size(target) ~= [N N]
    disp('Draw target');
    [~, target] = draw_set(grid);   % target_map = infty inside, -1 outside
    target = -target;               % target_map = -infty inside, 1 outside
    target(target<0) = -1;          % target map = -1 inside, 1 outisde
end

% ===== OBSTACLES ======
if size(obstacles) ~= [N N]
    obs_all = ones(N,N);
    nObs = input('How many obstacles to draw? ');
    
    while ~isscalar(nObs) || nObs < 0
        nObs = input('How many obstacles to draw? '); end
    
    for i = 1:nObs
        [~, obs_single] = draw_set(grid); % obs_map = infty inside, -1 outside
        obs_single = -obs_single;            % obs_map = -infty inside, 1 outside
        obs_single(obs_single<0) = 0;             % obs_map = 0 inside, -1 outside
        obs_all = obs_all.*obs_single;
    end
    obs_all(obs_all==0) = -1; % obs = -1 inside, 1 outside
    obstacles = obs_all;
end

% ===== ATTACKER POSITION ======
if size(A) ~= [n 2]
    disp('Specify attacker position'); A = ginput(n); end

% ===== DEFENDER POSITION =====
if size(D) ~= [n 2]
    disp('Specify defender position'); D = ginput(n); end

% ===== PACK OUTPUT =====
game = v2struct(n, domain, target, obstacles, A, D, grid, speed);
close(fig_draw)

end