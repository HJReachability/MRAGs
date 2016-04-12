clear all; close all;

%% Reach avoid slices
game = 'OLGameModified';
% game = 'midTarget_LObs_vJ';

load([game '_4DHJI'])
run(game)

xds = xd_init;
xas = xa_init;

f1 = figure;
f2 = figure;

plotReachSets(g, g2D, target2D, obs2D, data, ...
    xas, xds, captureRadius, f1, f2, dims_a, dims_d, dom_map)
        
return
%% Maximum matching

f3 = figure;
figure(f3)
title('Game Summary')

[~, hsd] = visualizeGame(g2D,target2D,obs2D,xas,xds, captureRadius, dom_map);

% Summary variables
Dgraph = cell(length(xd_init),1); % Set of attackers each defender can win against
for i = 1:length(xd_init) 
    Dgraph{i} = [];
    figure(f3)
    d = plot(xd_init{i}(1),xd_init{i}(2),'b*'); % Plot all player positions
    hold on
    
    for j = 1:length(xa_init)
        if i==1, figure(f3); a = plot(xa_init{j}(1),xa_init{j}(2),'*','color',[0.75 0 0]); end
        
        if ~in_reachset(g,data,[xa_init{j} xd_init{i}])     % Defender i wins against attacker j
            Dgraph{i} = [Dgraph{i} j];
            figure(f3); bip = plot([xd_init{i}(1) xa_init{j}(1)], [xd_init{i}(2) xa_init{j}(2)], 'b-');
        end
    end
end
axis(g2D.axis)

% Maximum matching
Dpairs = max_matching(Dgraph); % Maximum matching for defenders
figure(f3); hold on
for i = 1:size(Dpairs,1)
    dpos = xd_init{Dpairs(i,1)};
    apos = xa_init{Dpairs(i,2)};
    edge = [dpos; apos];
    
    maxm = plot(edge(:,1),edge(:,2),'b--','linewidth',2);
end

set(gca,'position',[0 0.1 0.75 0.75])
pos = get(gcf,'position');
set(gcf,'position',[pos(1) pos(2) 800 600]);

legend([hsd.ht hsd.ho hsd.hxds{1} hsd.hxas{1} bip maxm], {'Target','Obstacle','Defender','Attacker','Bipartite Graph','Maximum Matching'})
set(legend,'units','pixels','position',[550 200 250 150])