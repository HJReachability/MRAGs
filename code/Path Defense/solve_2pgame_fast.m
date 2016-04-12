function d_region = solve_2pgame_fast(game, xa, g, u_target, paths_set, paths_u, Rc, visualize)
% d_region = solve_2pgame_fast(game, xa, g, u_target, paths_set, paths_u, visualize)
% Solves the two-player reach-avoid game conservatively for the defender
% using strong path defense
%
% Input:
%   game    	- name of the game being considered
%   xa			- attacker position
% 	g 			- 2D grid structure
% 	u_target	- time-to-reach function from target set
% 	paths_set 	- Set of paths used to construct winning region
% 	paths_u		- time to reach functions from path end-points
%   Rc          - capture radius
% 	visualize 	- set to false to disable visualization of intermediate steps
%
% Outputs:
%   d_region    - defender winning region when computation ended
%
% Mo Chen, 2014-02-12
% clear all

if nargin<7, visualize=true; end % By default, visualize intermediate results

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

if visualize
    %% ---------------------------------------------------------------------------
    % FIGURES;
    main = figure;
    
    % show target
    [~, tard] = contour(g2D.xs{1}, g2D.xs{2}, target2D, [0 0]);
    set(tard,'color',[0 0.5 0],'linewidth',2)
    hold on
    
    % show obstacles
    if ~isempty(obs2D)
        [~, od] = contour(g2D.xs{1}, g2D.xs{2}, obs2D, [0 0]);
        set(od,'color','k','linewidth',2)
    end
    
    contour(g2D.xs{1}, g2D.xs{2}, dom_map, [0 0],'color','k','linewidth',2);
    
    % show attacker position
    ad = plot(xa(1), xa(2),'+','color',[0.5 0 0]);
    
    axis(g2D.axis);
    drawnow;
end
%% ATTACKER VALUE
u_A = compute_value(g2D, xa, velocitya, obs2D, dom_map);

%% Find paths
domain.bdry = dom_bdry;
domain.map = dom_map;
N_bdry_pts = round(length(paths_set));

i_dom_bdrys = round( linspace(1, length(paths_set), N_bdry_pts ) );
% i_dom_bdrys(2:2:end) = [];
% i_dom_bdrys(2:2:end) = [];
% i_dom_bdrys(2:2:end) = [];
% i_dom_bdrys(2:2:end) = [];
% i_dom_bdrys(2:2:end) = [];
% i_dom_bdrys(2:2:end) = [];
disp(['# of paths: ' num2str(length(i_dom_bdrys))])


for i = 250
% for i = 1:length(i_dom_bdrys)
    % ===== Find path of defense given boundary point ======
    i_dom_bdry = i_dom_bdrys(i);
    
%     if ~visualize, disp(['Path ' num2str(i) ' of ' num2str(length(i_dom_bdrys))]); end
%     tic
%     disp(i)
    path_i = choose_path(g2D, xa, obs2D, target2D, paths_set{i_dom_bdry}, dom_map);
%     toc
    
    if path_i>0
        % ===== Winning region for defender ======
%         disp('Computing winning region')
%         tic
        [d_pt, d_region, l_eps] = d_region_given_path(g2D, obs2D, velocityd, ...
            paths_set{i_dom_bdry}{path_i}, paths_u{i_dom_bdry}{path_i}{2}, u_A, Rc, domain.map);
%         toc
        
        if exist('d_region_union','var'),   d_region_union = shapeUnion(d_region, d_region_union);
        else                                d_region_union = shapeUnion(d_region, u_A - Rc);
        end
        
        if visualize
            % ===== Game summary figure =====
            if exist('g1','var'), delete(g1); end
            if exist('g2','var'), delete(g2); end
            if exist('g3','var'), delete(g3); end
            if exist('g4','var'), delete(g4); end
            if exist('g5','var'), delete(g5); end
            if exist('p','var'), delete(p); end
            if exist('d','var'), delete(d); end
            
            figure(main)
            [~, g1] = contour(g2D.xs{1},g2D.xs{2},paths_u{i_dom_bdry}{path_i}{2},[l_eps(1)-Rc l_eps(1)-Rc],'r');                    % First attacker winning region
            hold on
            [~, g2] = contour(g2D.xs{1},g2D.xs{2},paths_u{i_dom_bdry}{path_i}{1},[l_eps(2)-Rc l_eps(2)-Rc],'r');                    % Second attacker winning region
            [~, g3] = contour(g2D.xs{1},g2D.xs{2},d_region,[0 0],'g','linewidth',1.5);                            % Defender winning region for current line
            [~, g4] = contour(g2D.xs{1},g2D.xs{2},d_region_union,[0 0],'g','linewidth',1);                            % Defender winning region
            
            
            [~, g5] = contour(g2D.xs{1},g2D.xs{2},captureSet(g2D, d_pt,Rc),[0 0],'b','linewidth',1);  % defender capture radius
            % plot(path(1,1),path(2,1),'b*','markersize',15);          % path endpoint
            %         plot(paths_set{i_dom_bdry}{path_i}(1,end),paths_set{i_dom_bdry}{path_i}(2,end),'b.','markersize',20);      % path endpoint
            
            p = plot(paths_set{i_dom_bdry}{path_i}(1,:), paths_set{i_dom_bdry}{path_i}(2,:),'b-', 'LineWidth',2);          % path of defense
            d = plot(d_pt(1), d_pt(2),'gs', 'markersize',10,'markerfacecolor','g');         % best point to defend
            
            drawnow;
%             export_fig(['figs\dregion_' num2str(i)], '-png');
        end
    end
    
end

d_region = d_region_union;
% end