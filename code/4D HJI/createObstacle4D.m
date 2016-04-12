function [obs_a, obs_d] = createObstacle4D(g, obs, obs_type)
% function [obs_a, obs_d] = createObstacle4D(g, obs, obs_type)
% Creates a 4D obstacle based on fields in obs
%
% Inputs:
%   g           - grid structure
%   obs         - obstacle structure
%   obs_type    - obstacle shape
%
% Outputs:
%   obs_a       - obstacle in the space of attackers (visible when fixing
%                   defender's dimensions)
%   obs_d       - obstacle in the space of defenders (visible when fixing
%                   attacker's dimensions)
%
% Mo Chen, Sept. 26, 2013
%


if nargin<3, obs_type = 'rect'; end

switch obs_type
    case 'rect'
        % Obstacle in attacker space
        obs_a = shapeRectangleByCorners(g, ...
            [obs.xmin obs.ymin g.min(3) g.min(4)], ...
            [obs.xmax obs.ymax g.max(3) g.max(4)]);
        
        % Obstacle in defender space
        obs_d = shapeRectangleByCorners(g, ...
            [g.min(1) g.min(2) obs.xmin obs.ymin], ...
            [g.max(1) g.max(2) obs.xmax obs.ymax]);
        
        
%     case 'circ'
    otherwise
        error('Unknown obstacle type!')
end
end