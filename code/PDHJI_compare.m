clear all; close all;

game = 'OLGameModified';
% game = 'midTarget_LObs_vJ';

load([game '_dregions_Rc'])
load([game '_4DHJI'])
run(game)


fRA = figure;
spR = ceil(sqrt(length(xa_init)));
spC = ceil(length(xa_init)/spR);

data2D = cell(size(xa_init));
for i = 1:length(xa_init)
    subplot(2,2,i)
    [~, hs] = visualizeGame(g2D, target2D, obs2D, xa_init{i}, [], captureRadius, dom_map);
    [~, hPD] = contour(g2D.xs{1}, g2D.xs{2},  d_regions{i}, [0 0], 'r');
    
    [~, data2D{i}] = proj2D(g,[1 1 0 0], N2D, data, xa_init{i});
    [~, hHJI] = contour(g2D.xs{1}, g2D.xs{2},  data2D{i}, [0 0], 'b');
end
% return
legend([hPD, hHJI], 'Path Defense','4D HJI')

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
for i = 1:length(xa_init)
    subplot(spR,spC,i)
    set(gca,'position',subP_pos(i,:))
end
pos = get(gcf,'position');
set(gcf,'position',[100 200 800 600]);

% set(legend,'units','pixels','position',[570 200 200 50])
legend([hs.ht, hs.ho, hs.hxas{1}, hPD, hHJI], {'Target','Obstacle','Attacker','PD RA Slice','HJI RA Slice'})
set(legend,'units','pixels','position',[575 200 225 150])

%% -------------------------
% Compute areas
areaPD = zeros(size(xa_init));
areaHJI = zeros(size(xa_init));
for i = 1:length(xa_init)
    % path defense 2D slice area
    d_regions_af = zeros(N2D);
    d_regions_af(d_regions{i}<=0) = 1;
    areaPD(i) = compute_integral(g2D, d_regions_af);
    
    % HJI 2D slice area
    data2D_af = zeros(N2D);
    data2D_af(data2D{i}>=0) = 1;
    areaHJI(i) = compute_integral(g2D,data2D_af);
end

areaFrac = areaPD./areaHJI;