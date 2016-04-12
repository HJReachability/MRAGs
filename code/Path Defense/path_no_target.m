clear all; close all
%% LOAD GAME
gameName = 'basicgame';
[n, domain, target, obstacles, A, D, grid, speed, gameFig] = load_game(gameName);
[N,L,X,Y,x,y,infty] = v2struct(grid);
[main, dContour, tContour, oContour, aPlot, dPlot] = v2struct(gameFig);

delete(tContour); clear target; % if ignoring target
legend off

u_A = compute_value(grid, A, speed, obstacles, domain.map);
u_D = compute_value(grid, D, speed, obstacles, domain.map);
du = u_A - u_D;

%% INITIALIZE PLOTTING
contour(x,y,du,[0 0],'b','linewidth',3); % Voronoi

% Points on boundary coinciding with voronoi
[iX, iY] = ndgrid(1:N,1:N);
D_side_region = ones(N,N);
D_side_region(du>0) = -1;

D_dom_bdry = eval_u(D_side_region,X(domain.bdry(:,1)),Y(domain.bdry(:,2)),grid);

dom_bdry_D = domain.bdry(D_dom_bdry<0,:);

num_bdry_pts = size(dom_bdry_D,1);

prev_min_ind = 1;
safe_region_map = zeros(N,N);
iter = 0;
for i = 1:3:num_bdry_pts
    bdry_ind1 = i;
    bdry_ind2 = num_bdry_pts;
    
    prev_max_ind = num_bdry_pts;
    
    bdry_x1 = X(dom_bdry_D(bdry_ind1,1));
    bdry_y1 = Y(dom_bdry_D(bdry_ind1,2));
    
    inner_flag = 0;
    while prev_max_ind - prev_min_ind > 1
        bdry_x2 = X(dom_bdry_D(bdry_ind2,1));
        bdry_y2 = Y(dom_bdry_D(bdry_ind2,2));
        
%         plot(bdry_x1,bdry_y1,'b.','markersize',15)
%         plot(bdry_x2,bdry_y2,'bx','markersize',15)
        u_t = compute_value(grid,[bdry_x1 bdry_y1], speed, obstacles, domain.map);
        
        % Compute path of defense
        path = ComputeOptimalPath(u_t,bdry_x2, bdry_y2,speed,grid);
        if exist('p','var'), delete(p); end
        if exist('drContour','var'), delete(drContour); end
        if exist('dptPlot','var'), delete(dptPlot); end
        
        p = plot(path(1,:),path(2,:),'b-','linewidth',2);
        
        % Determine whether the path is strongly defendable
        [d_pt, d_region, l_eps, u_dn] = d_region_given_path(grid, obstacles, speed, path, u_t, u_A, domain.map);
        sd = eval_u(u_dn,D(1),D(2),grid)
        
        [~, drContour] = contour(x,y,d_region,[0,0],'g','linewidth',3);
        dptPlot =    plot(d_pt(1), d_pt(2),'gs');
        

        % Update next point on boundary
        if sd<=1
            disp('Strongly defendable')
            prev_min_ind = bdry_ind2;
            bdry_ind2 = floor((prev_max_ind+bdry_ind2)/2)
        else
            disp('Not strongly defendable')
            prev_max_ind = bdry_ind2;
            bdry_ind2 = ceil((prev_min_ind+bdry_ind2)/2)
        end
        
        if prev_max_ind - prev_min_ind == 1, inner_flag = 1; end
        % Define region
        if inner_flag
            
            safe_region_bdry = [dom_bdry_D(bdry_ind1:bdry_ind2,1) dom_bdry_D(bdry_ind1:bdry_ind2,2)];
            [iPathx, iPathy] = xy2inds(path(1,:)',path(2,:)',grid);
            safe_region_bdry = [safe_region_bdry; iPathx iPathy] ;
            safe_region_bdry = unique(safe_region_bdry,'rows');
            safe_region_map = safe_region_map + single(inpolygon(iX,iY,safe_region_bdry(:,1),safe_region_bdry(:,2)));
%             safe_region_map(safe_region_map==0)=-1;

            if exist('sC','var'), delete(sC); end
            [~,sC] = contour(x,y,safe_region_map,[0 0],'g','linewidth',3);
%             keyboard
        end 
        iter = iter+1;
        consFigName = ['..\fig\' gameName '_consFig_iter' num2str(iter) '_' num2str(bdry_ind1) '-' num2str(bdry_ind2)];
        saveas(main, consFigName, 'png')
        pause(0.01)
    end
end

% Compute safe area
safe_region_map(safe_region_map>0) = 1;
area_safe = compute_integral(grid,safe_region_map);

% Compute voronoi area
voronoi_region_map = zeros(N,N);
voronoi_region_map(du>0)= 1;
area_voronoi = compute_integral(grid,voronoi_region_map);

fraction_safe = area_safe/area_voronoi
