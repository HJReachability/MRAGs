function [in, val] = in_reachset(g,data,x)
% [in, val] = in_reachset(g,data,x)
% Checks weather the point x is inside the set described by data
% 
% Inputs:
%   g       - grid
%   data    - implicit function describing the set
%   x       - points to check; each row is a point
%
% Output:
%   in      - boolean; true if x is inside the set, false otherwise
%   val     - value of level set function at x
%
% Mo Chen, 2013-09-11
% 

% Create grid values
xs = cell(g.dim,1);
for i = 1:g.dim, xs{i} = linspace(g.min(i),g.max(i),g.N(i)); end

switch g.dim
    case 1, val = interpn(xs{1}, data, x);
    case 2, val = interpn(xs{1}, xs{2}, data, x(:,1),x(:,2));
    case 3, val = interpn(xs{1}, xs{2}, xs{3}, data,  x(:,1),x(:,2),x(:,3));
    case 4, val = interpn(xs{1},xs{2},xs{3},xs{4}, data, x(:,1),x(:,2),x(:,3),x(:,4));
    otherwise, error('Cannot check reach sets of dimension greater than 4!')
end

in = boolean(val<=0);
end