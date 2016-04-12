function [dom_bdry,dom_map] = OLGameModified_domain(grid)
% Function for drawing an arbitrary shape on grid
% Only works for square domains
%
% INPUT
%   grid:
%
% OUTPUT
%   dom_bdry - indices of boundary points
%   dom_map -  map of domain (N by N map; -1 means inside, 1 means outside)
%
% Mo Chen, 2013-06-07
%

% Set up figure based on grid

N = grid.N(1);
L = grid.max(1);
X = grid.vs{1};
Y = grid.vs{2};

x = grid.xs{1};
y = grid.xs{2};

xlim([-L L]); ylim([-L L]);

% Draw curve
% h = imfreehand(gca); 
% data = get(h);
% xydata=get(data.Children(4));
% xin=xydata.XData;
% yin=xydata.YData;

boundaryCorners = [-1 -1; ...
                    -0.1 -1; ...
                    -0.1 -0.3; ...
                    0.1 -0.3; ...
                    0.1 -1; ...
                    1 -1; ...
                    1 1; ...
                    -1 1
                    -1 -1];
xin = boundaryCorners(:,1)';
yin = boundaryCorners(:,2)';

% Convert to grid indices
[ix, iy] = xy2inds(xin,yin,grid);
dom_bdry = [ix' iy'];

dom_bdry_full = dom_bdry(1,:);
for i = 1:size(dom_bdry,1)-1
    if dom_bdry(i,1) == dom_bdry(i+1,1)
        sign_diff = sign(dom_bdry(i+1,2) - dom_bdry(i,2));
        fill_inds = dom_bdry(i,2)+sign_diff:sign_diff:dom_bdry(i+1,2);
        dom_bdry_full = [dom_bdry_full; ...
                dom_bdry(i,1)*ones(length(fill_inds),1) fill_inds'];
    end

    if dom_bdry(i,2) == dom_bdry(i+1,2)
        sign_diff = sign(dom_bdry(i+1,1) - dom_bdry(i,1));
        fill_inds = dom_bdry(i,1)+sign_diff:sign_diff:dom_bdry(i+1,1);
        dom_bdry_full = [dom_bdry_full; ...
                fill_inds' dom_bdry(i,2)*ones(length(fill_inds),1)];
    end
end

dom_bdry = dom_bdry_full;

[iX, iY] = ndgrid(1:N,1:N);
dom_map = -single(inpolygon(iX,iY,ix,iy));
dom_map(dom_map==0)=1;

end