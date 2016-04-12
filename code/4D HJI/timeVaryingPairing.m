% Computes optimal path for defenders and updates bipartite graph in while
% the game is progressing

%---------------------------------------------------------------------------
% Load game info and 4D HJI Data
clear all
% game = 'midTarget_LObs';
% game = 'nonconvexExample';
game = 'OLGameModified';
load([game '_4DHJI'])
run(game)

useSubplots = 0;

%---------------------------------------------------------------------------
% Compute the costate at every grid point
P = extractCostates(g,data); % Grid structure of costates

%---------------------------------------------------------------------------
% Initialize trajectory calculation
dt = 0.01;
Tf = 0.3;
N = length(xa_init);

xa_pos = zeros(N,2);
xd_pos = zeros(N,2);
xa_traj = cell(N,1);
xd_traj = cell(N,1);
xd_cap = cell(N,1);

for i = 1:N
    xa_pos(i,:) = xa_init{i};
    xd_pos(i,:) = xd_init{i};
    xa_traj{i} = xa_pos(i,:);
    xd_traj{i} = xd_pos(i,:);
    xd_cap{i} = captureSet(g2D, xd_pos(i,:), captureRadius);
end

%---------------------------------------------------------------------------
% Initialize graph figure
if useSubplots
    numPlots = 4;
    spR = ceil(sqrt(numPlots));
    spC = ceil(numPlots/spR);
    pdt = Tf/(numPlots-1);
    
    % Subplot parameters
    subP_size = 0.325;
    subP_xmin = 0.05;
    subP_ymin = 0.1;
    subP_xgap = 0.03;
    subP_ygap = 0.15;
    
    subP_pos = [subP_xmin               subP_ymin+subP_size+subP_ygap       subP_size subP_size;
        subP_xmin+subP_size+subP_xgap   subP_ymin+subP_size+subP_ygap   subP_size subP_size;
        subP_xmin                       subP_ymin                      subP_size subP_size;
        subP_xmin+subP_size+subP_xgap   subP_ymin                      subP_size subP_size];
else
    pdt = 1; % dummy variable
end

%---------------------------------------------------------------------------
% Compute time to reach function and gradient
if exist('obs_clrc','var'), u = compute_value(g2D, target2D, velocitya, obs_clrc);
else                        u = compute_value(g2D, target2D, velocitya, obs);
end

graphFig = figure;
plotNum = 1;
d = cell(N,1);
a = cell(N,1);
dcap = cell(N,1);
atraj = cell(N,1);
dtraj = cell(N,1);
maxm = cell(N,1);
for t = 0:dt:Tf
    % Calculate matching
    Dgraph = cell(N,1); % Set of attackers each defender can win against
    for i = 1:N
        Dgraph{i} = [];
        
        for j = 1:length(xa_init)
            if ~in_reachset(g,data,[xa_pos(j,:) xd_pos(i,:)])     % Defender i wins against attacker j
                Dgraph{i} = [Dgraph{i} j];
            end
        end
    end
    
    % Maximum matching
    quiet = true;
    Dpairs = max_matching(Dgraph, quiet); % Maximum matching for defenders
    
    Dpairs_full = zeros(N,2);
    Dpairs_full(:,1) = 1:N;
    for k = 1:N
        if find(Dpairs(:,1)==k), Dpairs_full(k,2) = Dpairs(Dpairs(:,1)==k,2); end
    end
    
    % Plot
    if t >= (plotNum-1)*pdt
        figure(graphFig);
        if useSubplots, subplot(spR,spC,plotNum); end
        
        for i = 1:N
            if ~useSubplots
                if ~isempty(d{i}), delete(d{i}); end
                if ~isempty(a{i}), delete(a{i}); end
                if ~isempty(dcap{i}), delete(dcap{i}); end
                if ~isempty(atraj{i}), delete(atraj{i}); end
                if ~isempty(dtraj{i}), delete(dtraj{i}); end
            end
            % Plot all player positions
            d{i} = plot(xd_pos(i,1),xd_pos(i,2),'b*');
            hold on
            a{i} = plot(xa_pos(i,1),xa_pos(i,2),'+','color',[0.75 0 0]);
            [~,dcap{i}] = contour(g2D.xs{1},g2D.xs{2},xd_cap{i}, [0 0],'color','b');
            
            % Plot all player trajectories
            atraj{i} = plot(xa_traj{i}(:,1),xa_traj{i}(:,2),'r:');
            dtraj{i} = plot(xd_traj{i}(:,1),xd_traj{i}(:,2),'b-');
        end
        
        % Plot maximum matching
        for k = 1:size(Dpairs,1)
            dpos = xd_pos(Dpairs(k,1),:);
            apos = xa_pos(Dpairs(k,2),:);
            match_edge = [dpos; apos];
            
            if ~useSubplots
                if ~isempty(maxm{k}), delete(maxm{k}); end
            end
            maxm{k} = plot(match_edge(:,1),match_edge(:,2),'b--','linewidth',2);
        end
        
        if useSubplots || ~exist('tard','var')
            % Show target
            tard = visualizeLevelSet(g2D, target2D, 'contour', 0);
            set(tard,'color',[0 0.5 0],'linewidth',2)
            hold on
            
            % Show obstacles
            od = visualizeLevelSet(g2D, obs, 'contour', 0);
            set(od,'color','k','linewidth',2)
        end
        
        title(['t=' num2str(t) ', m. size=' num2str(size(Dpairs,1))])
        axis(g2D.axis)
        
        if useSubplots
            plotNum = plotNum+1; 
        else
            directory = [game '_timeVarying1'];
            file_f = [directory '/' num2str(t*1000)];
            figure(graphFig)
            export_fig(file_f, '-png', '-transparent');
        end
        
    end
    
    % Calculate trajectories
    for i = 1:N
        % Determine which attacker to go for
        j = Dpairs_full(i,2);
        
        if j > 0 % If there is a pairing for this defender, then go for the assigned attacker
            
        else % Otherwise, go for the attacker that is closest
            ud = compute_value(g2D, xd_pos(i,:), velocityd, obs);
            vals = zeros(N,1);
            for k = 1:N
                vals(k) = eval_u(ud,xd_pos(i,1),xd_pos(i,2),g2D);
            end
            [~, j] = max(vals);
        end
        

        [dira, dird] = HJIDirection(g,P,xa_pos(j,:), xd_pos(i,:));
        ud = velocityd*dird;
        
        dira = shortestPathDirection(g2D, u, xa_pos(i,:));
        
        ua = velocitya*dira;        

        xa_pos(i,:) = xa_pos(i,:) + ua*dt;
        xd_pos(i,:) = xd_pos(i,:) + ud*dt;
        xd_cap{i} = captureSet(g2D,xd_pos(i,:),captureRadius);
        
        xa_traj{i} = [xa_traj{i}; xa_pos(i,:)];
        xd_traj{i} = [xd_traj{i}; xd_pos(i,:)];
    end
end

% Set plot size and legend
figure(graphFig)
pos = get(gcf,'position');
set(gcf,'position',[pos(1) pos(2) 800 600])
for i = 1:numPlots
    subplot(spR,spC,i)
    set(gca,'position',subP_pos(i,:))
end
legend([tard od a d maxm atraj dtraj], {'Target','Obstacle','Attacker',...
    'Defender','Max. Matching','Attacker Traj.','Defender Traj.'})
set(legend,'units','pixels','position',[575 200 200 150])