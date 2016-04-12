function P = extractGradient(grid, data)
% p = extractCostates(grid, data)
%
% Estimates gradiate of function data

P = cell(grid.dim,1);
for i = 1:grid.dim
    [derivL, derivR] = upwindFirstWENO5(grid, data, i);
    P{i} = (derivL + derivR)/2;
end

end