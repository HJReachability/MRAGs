function [d_pt, d_region, l_eps] = d_region_given_path(grid, obs, velocityd, path, u_t, u_A, Rc, dom_map)
% [d_pt, d_region, l_eps] = d_region_given_path(grid, obs, velocityd, path, u_t, u_A, dom_map)
% Given a path of defense, determines the winning region for defender
%
% INPUTS:
% grid:
% obs:
% velocityd
% path:     path of defense
% u_t:      value function from path(:,end)
% u_A:      value of attacker
% dom_map:  domain map
% Rc:       capture radius
%
% OUTPUTS:
%   d_pt:       best point to defend
%   d_region:   winning region for defender induced by d_pt
%   l_eps:      distance from boundary end-points to d_pt
%
% Mo Chen, 2014-02-07
%

checkGrid(grid);

l = eval_u(u_t, path(1,1), path(2,1), grid); % path length

% attacker's value to end points
uA_ep1 =  eval_u(u_A, path(1,end), path(2,end), grid);
uA_ep2 =  eval_u(u_A, path(1,1), path(2,1), grid);

% value of attacker such that attacker is equidistant to the two attacker
% winning regions
uA_eqwin = 0.5*(uA_ep1 + uA_ep2 - l);

% length from end-points
l_ep1 = uA_ep1 - uA_eqwin;
l_ep2 = uA_ep2 - uA_eqwin;
l_eps = [l_ep1 l_ep2];

% values on the path
u_path = eval_u(u_t,path(1,:),path(2,:),grid);

% optimal point on the path to defend
[~, besti] = min(abs(l_ep1 - u_path));
d_pt = [path(1,besti) path(2,besti)];

% REGION IN WHICH DEFENDER CAN DEFEND THE PATH
% value function from optimal point of defense
u_d = compute_value(grid, d_pt, velocityd, obs, dom_map);
d_region = u_d - uA_eqwin - Rc;

end