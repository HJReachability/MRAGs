function xd_cap = captureSet(g, xd,captureRadius)
% xd_cap = captureSet(g, xd,captureRadius)
% Computes the level set representation of a circular capture region
%
% Inputs:
%   g               - grid structure
%   xd              - defender position
%   captureRadius   - capture radius
%
% Output:
%   xd_cap          - level set representation of capture region
%
%

xd_cap = (g.xs{1} - xd(1)).^2 + (g.xs{2} - xd(2)).^2 - captureRadius^2;

end
