function d_region = solve_2pgame(game, xa, xd, g)
% [d_flag, d_region] = solve_2pgame(gameName)
% Solves the two-player reach-avoid game conservatively for the defender
% using strong path defense
%
% Input:
%   gameName    - name of the game being considered; see load_game.m
%
% Outputs:
%   d_flag      - =1 if defender can win; =0 otherwise
%   d_region    - defender winning region when computation ended
%
% Mo Chen, 2013-06-28
% clear all

%---------------------------------------------------------------------------
% GRID
if nargin<4
    Nx = 45;
    
    % Create the computation grid.
    g.dim = 4;
    g.min = [  -1; -1; -1; -1];
    g.max = [ +1; +1; +1; +1 ];
    g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate };
    % Roughly equal dx in x and y (so different N).
    g.N = [ Nx; Nx; Nx; Nx ];
    
    g = processGrid(g);
end
infty = 1e6;

%% LOAD GAME
% run OLGameModified
% run midTarget_LObs
run(game);
if nargin<2, xa = xa_init{1}; end

%% ---------------------------------------------------------------------------
% FIGURES;
main = figure;

% show target
[~, tard] = contour(g2D.xs{1}, g2D.xs{2}, target2D, [0 0]);
set(tard,'color',[0 0.5 0],'linewidth',2)
hold on

% show obstacles
if ~isempty(obs)
    [~, od] = contour(g2D.xs{1}, g2D.xs{2}, obs, [0 0]);
    set(od,'color','k','linewidth',2)
end

% show attacker position
ad = plot(xa(1), xa(2),'*','color',[0.5 0 0]);

axis(g2D.axis);
drawnow;

%% ATTACKER VALUE
u_A = compute_value(g2D, xa, velocitya, obs, dom_map);

%% TARGET VALUE
u_target = compute_value(g2D, target2D, velocitya, obs, dom_map);

%% Find paths
domain.bdry = dom_bdry;
domain.map = dom_map;
N_bdry_pts = 50;

i_dom_bdrys = round( linspace(1,size(dom_bdry,1), N_bdry_pts ) );

for i = 1:length(i_dom_bdrys)
% for i = 25
    % ===== Find path of defense given boundary point ======
    i_dom_bdry = i_dom_bdrys(i);
    
    disp('Calculating path to defend')
    tic; [path, u_eps, i_end_pts] = find_path(g2D, target2D, obs, velocitya, ...
        i_dom_bdry, domain, xa, u_target); toc
    
    plot(g2D.vs{1}(dom_bdry(i_dom_bdry,1)), ...
        g2D.vs{2}(dom_bdry(i_dom_bdry,2)),'bo','markersize',20);      % path endpoint
    drawnow
    
    if ~isempty(path) % if a path is found
        % ===== Winning region for defender ======
        tic; [d_pt, d_region, l_eps] = d_region_given_path(g2D, obs, velocitya, path, u_eps{1}, u_A, domain.map); toc
        
        if exist('d_region_union','var'),   d_region_union = d_region + d_region_union;
        else                                d_region_union = d_region;
        end
        d_region_union(d_region_union>0) = infty;
        
        % ===== Game summary figure =====
        if exist('g1','var'), delete(g1); end
        if exist('g2','var'), delete(g2); end
        if exist('g3','var'), delete(g3); end
        if exist('g4','var'), delete(g4); end
        if exist('p','var'), delete(p); end
        if exist('d','var'), delete(d); end
        
        figure(main)
        [~, g1] = contour(g2D.xs{1},g2D.xs{2},u_eps{1},[l_eps(1) l_eps(1)],'r');                    % First attacker winning region
        hold on
        [~, g2] = contour(g2D.xs{1},g2D.xs{2},u_eps{2},[l_eps(2) l_eps(2)],'r');                    % Second attacker winning region
        [~, g3] = contour(g2D.xs{1},g2D.xs{2},d_region,[0 0],'g','linewidth',1.5);                            % Defender winning region for current line
        [~, g4] = contour(g2D.xs{1},g2D.xs{2},d_region_union,[0 0],'g','linewidth',3);                            % Defender winning region
        
        % plot(path(1,1),path(2,1),'b*','markersize',15);          % path endpoint
        plot(path(1,end),path(2,end),'b.','markersize',20);      % path endpoint
        
        p = plot(path(1,:), path(2,:),'b-', 'LineWidth',2);          % path of defense
        d = plot(d_pt(1), d_pt(2),'gs', 'markersize',10,'markerfacecolor','g');         % best point to defend
        
        pause(0.01)
        drawnow;
    end
end

d_region = d_region_union;
% end