clear all; close all;

% Compile mex file
mex('..\..\..\FMM\code\c version\mexEikonalFMM.cpp')
mex('..\..\..\FMM\code\c version\shortestPathc.cpp')
%% GRID
Nx = 45;

% Create the computation grid.
g.dim = 4;
g.min = [  -1; -1; -1; -1];
g.max = [ +1; +1; +1; +1 ];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate };
% Roughly equal dx in x and y (so different N).
g.N = [ Nx; Nx; Nx; Nx ];

g = processGrid(g);

%% ----- Load Game -----
game = 'OLGameModified';
% game = 'midTarget_LObs_vJ';
% game = 'nonconvexExample';
run(game);

N_bdry_pts = size(dom_bdry,1);
i_dom_bdrys = round( linspace(1,size(dom_bdry,1), N_bdry_pts ) );
%     i_dom_bdrys(2:2:end) = [];
%     i_dom_bdrys(2:2:end) = [];
%     i_dom_bdrys(2:2:end) = [];
%     i_dom_bdrys(2:2:end) = [];
%     i_dom_bdrys(2:2:end) = [];
%     i_dom_bdrys(2:2:end) = [];
disp(['# of paths: ' num2str(length(i_dom_bdrys))])

u_target = compute_value(g2D,target2D,velocityd,obs2D,dom_map);
% if 0
%% ----- Compute paths of defense for the current game setup -----
if exist([game '_paths.mat'],'file')
    disp('Loading previously calculated data')
    load([game '_paths.mat'])
else
    
   
    domain.bdry = dom_bdry;
    domain.map = dom_map;
    
    paths_set = cell(length(i_dom_bdrys), 1);
    
    figure(1);
    contour(g2D.xs{1},g2D.xs{2},obs2D,[0 0],'linecolor','k','linewidth',3)
    hold on
    contour(g2D.xs{1},g2D.xs{2},target2D,[0 0],'linecolor','g','linewidth',3)
    contour(g2D.xs{1},g2D.xs{2},dom_map,[0 0],'linecolor','k','linewidth',3)
    p = cell(2,1);
     tic
    for i = 1:length(i_dom_bdrys)
        % for i = 25
        % ===== Find path of defense given boundary point ======
        i_dom_bdry = i_dom_bdrys(i);
        
%         tic; 
        paths = find_paths(g2D, target2D, obs2D, velocitya, ...
            i_dom_bdry, domain, u_target); 
%         toc
        
        paths_set{i} = paths;
%         
        for j = 1:length(paths)
            % ===== summary figure =====
            if ~isempty(p{j}), delete(p{j}); end
            
            % plot(path(1,1),path(2,1),'b*','markersize',15);          % path endpoint
            if ~isempty(paths{j})
                plot(paths{j}(1,end),paths{j}(2,end),'b.','markersize',20);      % path endpoint
                p{j} = plot(paths{j}(1,:), paths{j}(2,:),'b-', 'LineWidth',2);          % path of defense 1
            end
            
            drawnow
%             export_fig(['figs/paths_' num2str(i)],'-png')
        end

    end
    paths_set_time = toc
    % Get rid of empty paths
    paths_set(cellfun('isempty',paths_set)) = [];
    for i = 1:length(paths_set)
        paths_set{i}(cellfun('isempty',paths_set{i})) = [];
    end
    
    save([game '_paths.mat'],'paths_set')
end

% return

tic
%% For each path, compute value functions from end points
if ~exist('paths_u','var')
    disp('Computing value functions')
    paths_u = cell(length(paths_set),1);
    for i = 1:length(paths_set)
%         tic;
%         disp(['Path ' num2str(i)])
        paths_u{i} = cell(length(paths_set{i}),1);
        
        for j = 1:length(paths_set{i})
            paths_u{i}{j} = cell(2,1);
            paths_u{i}{j}{1} = compute_value(g2D,paths_set{i}{j}(:,1)',velocitya, obs2D, dom_map);
            paths_u{i}{j}{2} = compute_value(g2D,paths_set{i}{j}(:,end)',velocitya, obs2D, dom_map);
        end
%         toc
    end
%     save([game '_paths.mat'],'paths_u','-append')

end
paths_u_time = toc
% return
% end

%% For testing
if 0
figure;
contour(g2D.xs{1},g2D.xs{2},obs2D,[0 0],'linecolor','k','linewidth',3)
hold on
contour(g2D.xs{1},g2D.xs{2},target2D,[0 0],'linecolor','g','linewidth',3)
p = cell(2,1);
for i = 1:length(paths_set)
    if ~isempty(paths_set{i})
        for j = 1:length(paths_set{i})
            % ===== summary figure =====
            
            if ~isempty(p{j}), delete(p{j}); end
            plot(paths_set{i}{j}(1,end),paths_set{i}{j}(2,end),'b.','markersize',20);      % path endpoint
            p{j} = plot(paths_set{i}{j}(1,:), paths_set{i}{j}(2,:),'b-', 'LineWidth',2);          % path of defense 1
%             pause(0.1)
            drawnow
        end
    end
end
end

%% ----- Compute n winning regions n two-player games -----
% assumes same speed
% if exist([game '_dregions_Rc.mat'],'file')
%     load([game '_dregions_Rc.mat'])
% else
%     d_regions = cell(Na,1);
% end
Dgraph = cell(Nd,1);
solve_2p_time = zeros(Na,1);
visualize = 1;

for i = 1:Na
    xa = xa_init{i};
%     if ~exist([game '_dregions.mat'],'file')
        
        tic
        d_regions{i} = solve_2pgame_fast(game, xa, g, u_target, paths_set, paths_u, captureRadius, visualize); toc
        solve_2p_time(i) = toc;
%     end
    
    for j = 1:Nd
        plot(xd_init{j}(1),xd_init{j}(2),'b*'); hold on
        in_d_region = eval_u(d_regions{i},xd_init{j}(1), xd_init{j}(2), g2D);
        
        if in_d_region <= 0
            Dgraph{j} = [Dgraph{j} i];
        end
    end
end
% if ~exist([game '_dregions.mat'],'file')
%     save([game '_dregions.mat'],'d_regions');
% end

%% ----- Plot reach-avoid slices -----
fRA = figure;
spR = ceil(sqrt(Na));
spC = ceil(Na/spR);
for i = 1:Na
    subplot(spR, spC, i)
    [~, hs] = visualizeGame(g2D,target2D,obs2D,xa_init(i),xd_init,0,dom_map);
    [~, hdr] = contour(g2D.xs{1}, g2D.xs{2}, d_regions{i}, [0 0], 'linecolor','r');
end

%% ----- Plot initial pairing -----
matchFig = figure; hold on
[~, hsm] = visualizeGame(g2D,target2D,obs2D,xa_init,xd_init,0,dom_map);
for i = 1:length(Dgraph)
    for j = 1:length(Dgraph{i})
        edge = [xd_init{i}; xa_init{Dgraph{i}(j)}];
        hm = plot(edge(:,1), edge(:,2), 'b-');
    end
end

%% ----- Plot maximum matching -----
pairs = max_matching(Dgraph);
figure(matchFig); hold on
for i = 1:size(pairs,1)
    match_edge = [xd_init{pairs(i,1)}; xa_init{pairs(i,2)}];
    hmm = plot(match_edge(:,1),match_edge(:,2),'b--','linewidth',2);
end



%% ---------------------------------------------------------------------------
% Figure formatting for winning regions
% Subplot positions
% Subplot spacing!
subP_size = 0.325;
subP_xmin = 0.05;
subP_ymin = 0.1;
subP_xgap = 0.03;
subP_ygap = 0.15;

subP_pos = [subP_xmin               subP_ymin+subP_size+subP_ygap       subP_size subP_size;
    subP_xmin+subP_size+subP_xgap   subP_ymin+subP_size+subP_ygap   subP_size subP_size;
    subP_xmin                       subP_ymin                      subP_size subP_size;
    subP_xmin+subP_size+subP_xgap   subP_ymin                      subP_size subP_size];

figure(fRA)
for i = 1:Na
    subplot(spR,spC,i)
    set(gca,'position',subP_pos(i,:))
end
pos = get(gcf,'position');
set(gcf,'position',[100 200 800 600]);

legend([hs.ht hs.ho hs.hxds{1} hs.hxas{1} hdr], {'Target','Obstacle','Defender','Attacker','Defender Winning Region'})
set(legend,'units','pixels','position',[550 200 250 150])

% Figure formatting for summary figure
figure(matchFig)
set(gca,'position',[0 0.1 0.75 0.75])
pos = get(gcf,'position');
set(gcf,'position',[100 200 800 600]);
axis square

legend([hsm.ht hsm.ho hsm.hxds{1} hsm.hxas{2} hm hmm], {'Target','Obstacle','Defender','Attacker','Bipartite Graph','Maximum Matching'})
set(legend,'units','pixels','position',[550 200 250 150])