function J = jacob0(S,q)
    % your code here
    
    % If necessary, you calculate the homogeneous transformation associated with a twist V and displacement omega with:
    % T = twist2ht(V,omega);
    [S_row,S_col] = size(S);
    J =[];
    T = eye(4);
    for i = 1:S_col
        T = T * twist2ht(S(:,i),q(i));
        Adj_S = adjoint(S(:,i),T);
        J = [J Adj_S];
    end
    % You can also calculate the adjoint transformation of a twist V w.r.t. a homogeneous transformation matrix T with:
    % adjoint(V,T)
end