function pairs = max_matching(D,quiet)
% pairs = max_matching(D,quiet)
% Find the maximum matching given a bipartite graph
%
% Input:
%   D       - cell structure; D{i} contains the attackers that defender i 
%               can defend
%   quiet   - suppresses cvx output
% 
% Output: 
%   pairs   - pairings from maximum matching; each row is a pair
%
% Mo Chen, 2013-06-28
%

if nargin<2, quiet = false; end

%% ----- Players (Input data) -----
num_players = length(D);

%% ----- Assign Edges -----
numE = length([D{:}]);
if numE == 0
    pairs = zeros(0,2);
    return;
end

E = zeros(numE,2);
E_num = 0;
for i = 1:num_players
    for j = 1:length(D{i})
        % Defender i's vertex is labeled 2*i
        % Attacker i's vertex is labeled 2*i-1
        % Each row of E is a [defender vertex, attacker vertex] pair
        E_num = E_num+1;
        E(E_num,:) = [2*i 2*D{i}(j)-1];
    end
end

%% ----- Vertices -----
numV = max(E(:));

V = (1:numV)';
 
%% ----- Edge matrix -----
Emat = zeros(numV);
for i = 1:numE, Emat(E(i,1), E(i,2)) = 1; end
Emat = Emat + Emat';

%% ----- Find maximum matching using LP -----
if quiet cvx_begin quiet
else cvx_begin; end
variable xe(numE,1)
maximize sum(xe)

subject to
% -- Constraint 1 --
for i = 1:numV % Go through all vertices
    eu = find(Emat(i,:) == 1);
    Einds = [];
    for j = eu
        [~, Eind] = ismember([i j], E, 'rows');
        if Eind > 0, Einds = [Einds; Eind]; end
        
        [~, Eind] = ismember([j i], E, 'rows');
        if Eind > 0, Einds = [Einds; Eind]; end
    end
    
    sum(xe(Einds))<=1;
end
% -- Constraint 2 --
xe >= 0;
xe <= 1;
% -- End Constraints --
cvx_end

%% ----- Convert LP solution to defender-attacker pairs -----
pairs = E(xe==1, :);
pairs(:,1) = pairs(:,1)/2;
pairs(:,2) = (pairs(:,2)+1)/2;

end