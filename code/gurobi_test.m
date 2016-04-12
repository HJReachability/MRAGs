%%
clear all;

% names = {'u12'; 'u13'; 'u21'; 'u23'; 'u31'; 'u32'};
% 
% model.A = sparse([1 1 0 0 0 0; 0 0 1 1 0 0; 0 0 0 0 1 1; ...
%     1 0 1 0 0 0; 0 1 0 0 1 0; 0 0 0 1 0 1]);
% model.obj = [1 1 1 1 1 1];
% model.rhs = [1; 1; 1; 1; 1; 1];
% model.sense = '===>>>';
% model.vtype = 'B';
% model.modelsense = 'max';
% 
% params.outputflag = 0;
% params.resultfile = 'mip1.lp';
% 
% result = gurobi(model, params);
% 
% disp(result)
% 
% for v=1:length(names)
%     fprintf('%s %d\n', names{v}, result.x(v));
% end
% 
% fprintf('Obj: %e\n', result.objval);

%%
% clear all;
% 
% names = {'u12'; 'u13'; 'u21'; 'u23'; 'u24'; 'u31'; 'u32'; 'u34'; 'u42'; 'u43'};
% 
% model.A = sparse([  1 1 0 0 0 0 0 0 0 0; ... % Aircrafts can only avoid one other aircraft
%                     0 0 1 1 1 0 0 0 0 0; ...
%                     0 0 0 0 0 1 1 1 0 0; ...
%                     0 0 0 0 0 0 0 0 1 1; ...
%                     1 0 1 0 0 0 0 0 0 0; ... % Each aircraft must be safe
%                     0 1 0 0 0 1 0 0 0 0; ... 
%                     0 0 0 1 0 0 1 0 0 0; ...
%                     0 0 0 0 1 0 0 0 1 0; ...
%                     0 0 0 0 0 0 0 1 0 1]);
%     
% model.obj = ones(length(names),1);
% model.rhs = [   1; 1; 1; 1; ...
%                 1; 1; 1; 1; 1];
% model.sense = '====>>>>>';
% model.vtype = 'B';
% model.modelsense = 'max';
% 
% params.outputflag = 0;
% params.resultfile = 'mip1.lp';
% 
% result = gurobi(model, params);
% 
% disp(result)
% 
% for v=1:length(names)
%     fprintf('%s %d\n', names{v}, result.x(v));
% end
% 
% fprintf('Obj: %e\n', result.objval);

%%
clear all;

% === Graphical structure ===
n = 6;
Gall = zeros(n);
Gall(1,2) = 1;
Gall(2,3) = 1;
Gall(3,1) = 1;
Gall(1,4) = 1;

% G(4,1) = 1;
% G(1,5) = 1;
% G(2,5) = 1;
Gall = Gall+Gall';
Gr = Gall;
nonP = find(~any(Gall,2));
Gr(~any(Gr,2),:) = [];
Gr(:,~any(Gr,1)) = [];
nr = size(Gr,1);

% -- remove diagonal --
% G = G';
Gr(logical(eye(nr))) = [];
Gr = reshape(Gr',nr-1,nr)';

% -- assemble variable labels for Gurobi --
names = cell(nr);
uTag = cell(nr);
for i = 1:nr
    for j = 1:nr
        names{i,j} = ['u' num2str(i) num2str(j)];
        uTag{i,j} = [i j];
    end
end
names = names';
uTag = uTag';
names(logical(eye(nr))) = [];
uTag(logical(eye(nr))) = [];
uTag = reshape([uTag{:}]',2,nr*(nr-1))';

% -- assemble optimization matrix for Gurobi (control) --
avoidMat = zeros(nr, nr*(nr-1));
for i = 1:nr
    for j = 1:nr
        avoidMat(i, (i-1)*(nr-1)+1:i*(nr-1)) = Gr(i,:);
    end
end

% -- assemble optimization matrix for Gurobi (safety) --
safeMat = zeros(nr*(nr-1)/2, nr*(nr-1));
for i = 1:nr
    for j = i+1:nr
        row = nr*(nr-1)/2 - (nr-i)*(nr-i+1)/2 + j-i;
        safeMat(row, uTag(:,1) == i & uTag(:,2) == j) = 1;
        safeMat(row, uTag(:,1) == j & uTag(:,2) == i) = 1;
    end
end
    
modelMat = [avoidMat; safeMat];

model.A = sparse(modelMat);
    
model.obj = ones(length(names),1);
model.rhs = ones(size(modelMat,1),1);

sense = cell(size(modelMat,1),1);
for i = 1:size(modelMat,1)
    if i <= size(avoidMat,1)
        sense{i} = '=';
    else
        sense{i} = '>';
    end
end

model.sense = char(sense)';
model.vtype = 'B';
model.modelsense = 'max';

params.outputflag = 0;
params.resultfile = 'mip1.lp';

result = gurobi(model, params);

disp(result)

% for v=1:length(names)
%     fprintf('%s %d\n', names{v}, result.x(v));
% end
% 
% fprintf('Obj: %e\n', result.objval);

% Display results
resultMatr = reshape(result.x,nr-1,nr)';
resultMat = zeros(nr);
for i = 1:nr
    resultMat(i,:) = [resultMatr(i,1:i-1) 0 resultMatr(i,i:end)];
end

for i = 1:nr
    for j = 1:nr
        if Gall(i,j) && resultMat(i,j)
            disp([num2str(i) ' avoids ' num2str(j)])
        end
    end
end

for i = 1:length(nonP)
    disp([num2str(nonP(i)) ' is not part of graph'])
end