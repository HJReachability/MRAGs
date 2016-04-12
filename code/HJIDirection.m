function [dira, dird] = HJIDirection(g,P,xa,xd)
% [ua, ud] = HJIStrategy(g,P,xa,xd)
% Calculates the optimal control in the HJI sense
% 
% Inputs:
%   g       - 4D grid structure
%   P       - array of gradients of the reachable set
%   xa, xd  - attacker and defender positions (size must be [1 2])
%
% Outputs
%   ua, ud  - unit row vectors representing control input direction
% 
% Mo Chen, Oct. 7, 2013
%

p = calculateCostate(g,P,[xa xd]);

pa = p(1:2);
pd = p(3:4);

small = 1e-9;
if norm(pa)>small,  dira = -pa / norm(pa);
else                dira = [0 0]; end

if norm(pd)>small,  dird = pd / norm(pd); 
else                dird = [0 0]; end

end