function J = jacob0(S,q)
% Your code here
    trans_mat = {};
    T = eye(4);
    % If necessary, you calculate the homogeneous transformation associated with a twist V and displacement omega with:
    for i = 1:width(S)
        T = T*twist2ht(S(1:end,i),q(i));
        trans_mat{i} = T;
    end
    j = [];
    for i = 1:width(S)
        j(i,1:6) = adjoint(S(1:end,i),trans_mat{i});
    end
    J = j';
end