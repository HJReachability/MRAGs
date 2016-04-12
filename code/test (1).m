A = rand(5,5);
b = rand(5,1);

cvx_begin
variable x(5,1)

minimize norm(A*x - b)

subject to
norm(x) <= 1


cvx_end

