function [path_segment, path_segment_i] = path_obstacle(path, p, u_A, u_D, u_nt,grid)
% path_segment = path_obstacle(path, u_A, u_D, u_nt)
% Computes the segment of the path that is effectively an obstacle to the
% attacker
%
% Inputs:
%   path    - (x,y) points of the path
%   p       - defender's first point of arrival to path
%   u_A     - time to reach function of attacker
%   u_D     - time to reach function of defender
%   u_nt    - time to reach function of anchor point
%
% Output:
%   path_segment    -   .xy: (x,y) coordinates of the segment
%
% Mo Chen, 2013-06-28
%

d_pathA = eval_u(u_A,path(1,:),path(2,:),grid); % Distance from path to A
d_Dp = eval_u(u_D,p(1),p(2),grid);              % Distance from D to p
d_pathAset = d_pathA - d_Dp;                    % Distance from path to Dp level set around A

d_ep = eval_u(u_nt,p(1),p(2),grid);                  % Distance from p to anchor point
d_epath = eval_u(u_nt,path(1,:),path(2,:),grid);     % Distance from path to anchor point
d_ppath = abs(d_epath-d_ep);                         % Distance from path to p

diff_d = d_pathAset - d_ppath;                  % Compare distance to Dp level set around A and distance to p

path_segment = path(:,diff_d>=0);
path_segment_i = [find(diff_d>=0,1,'first') find(diff_d>=0,1,'last')];
% keyboard
end