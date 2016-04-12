close all; clear all;

%% Game information
game = 'midTarget_LObs_fastA';
xA = [-0.6 0.4];
xD1 = [-0.9 -0.8];
xD2 = [0.8 0.8];

% game = 'LTarget_noObs';
% xA = [-0.5 0.5];
% xD1 = [0.6 0.5];
% xD2 = [-0.5 -0.6];

load([game '_4DHJI'])
run(game);

prev_obs = obs2D;

%% First defender
figure;
[AWinSet, uD1, new_obs] = blockPath(game, xA, xD1, prev_obs);

keyboard
%% Second defender
prev_obs = new_obs;

figure;
[AWinSet, uD2, new_obs] = blockPath(game, xA, xD2, prev_obs);

%% Play out game with both defenders moving simultaneously
