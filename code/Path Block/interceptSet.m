function iSet = interceptSet(g2D, obs2D, dom_map, uA, path, velocityd, captureRadius)
% iSet = interceptSet(g2D, obs2D, dom_map, uA, path, velocityd, captureRadius)
%
% Computes the set in which the defender can intercept some path of the
% attacker
%
% Inputs:
%   g2D         - 2D grid structure
%   obs2D       - 2D obstacles
%   dom_map     - domain
%   uA          - attacker's time to reach all points in state space
%   path        - path being considered
%   velocityd   - defender speed
%   captureRadius -
%
% Outputs:
%   iSet        - set in which the defender can intercept some path of the
%                   attacker (<=0 means in the set)
%
% Mo Chen, 2014-04-04
   
for i = 1:size(path,2) % For every point on the path
    v = compute_value(g2D, path(:,i), velocityd, obs2D, dom_map);
    tpath = eval_u(g2D, uA, path(1,:));
    v = v-tpath-captureRadius;
    
    if i == 1,  iSet = v;
    else        iSet = shapeUnion(iSet,v);
    end
end

iSet = -iSet;
end