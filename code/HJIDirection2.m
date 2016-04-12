function [dira, dird] = HJIDirection2(g,phi,xa,xd)
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

% dx vectors
xp = cell(4,1); 
for i = 1:length(xp)
    xp{i} = zeros(1,4);
    xp{i}(i) = g.dx(i);
end

% Calculate values of a simplex and calculate gradient
[~, phix] = in_reachset(g,phi,[xa xd]);

phixp = cell(4,1);
phixm = cell(4,1);
p_plus = cell(4,1);
p_minus = cell(4,1);
p_up = zeros(1,4);
p_down = zeros(1,4);
for i = 1:length(phixp)
    % Calculate values of a simplex
    [~, phixp{i}] = in_reachset(g,phi,[xa xd]+xp{i});
    [~, phixm{i}] = in_reachset(g,phi,[xa xd]-xp{i});
    
    % Calculate gradient
    p_plus{i} = phixp{i} - phix;    % Right gradient
    p_minus{i} = phix - phixm{i};   % Left gradient
    
    if p_plus{i} >= p_minus{i}
        p_up(i) = p_plus{i};
        p_down(i) = p_minus{i};
    else
        p_up(i) = p_minus{i};
        p_down(i) = p_plus{i};
    end
end

% Normalize directions
dira = -p_down(1:2)/norm(p_down(1:2));
dird = p_up(3:4)/norm(p_up(3:4));

end