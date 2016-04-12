function path_i = choose_path(grid, xa, obs, target, paths, dom_map)
% path_i = choose_path(grid, xa, obs, target, paths, dom_map)
%
% Chooses a path of defense that blocks the attacker from the target
% 
% grid:     grid information
% xa:       attacker position
% obs:      obstacle
% dom_map:  map of domain (N by N)

checkGrid(grid);

NPath = length(paths);
paths_ix = cell(NPath,1);
paths_iy = cell(NPath,1);

for i = 1:NPath
    % Convert paths to indices
    [paths_ix{i}, paths_iy{i}] = xy2inds(paths{i}(1,:)',paths{i}(2,:)',grid);
    
    obsp = obs;
    % Make the path an obstacle
    for j = 1:length(paths_ix{i})
        obsp(paths_ix{i}(j), paths_iy{i}(j)) = -1;
        obsp(min([paths_ix{i}(j)+1 grid.N(1)]), paths_iy{i}(j)) = -1;
        obsp(max([paths_ix{i}(j)-1 1]), paths_iy{i}(j)) = -1;
        obsp(paths_ix{i}(j), min([paths_iy{i}(j)+1 grid.N(2)])) = -1;
        obsp(paths_ix{i}(j), max([paths_iy{i}(j)-1 1])) = -1;
    end
    
    % Check if the path blocks attacker from target
    u_targetp = compute_value(grid, target, 1, obsp, dom_map);
    aValue = eval_u(u_targetp, xa(1), xa(2), grid);

    if aValue > 5e2
        path_i = i;
        return
    end
end

path_i = 0;

end