function J_a = jacoba(S,M,q)    
    % your code here
        % your code here
    J_s = jacob0(S,q);
    T = fkine(S,M,q,'space');
    
    J_a = -skew(T(1:3,4))* J_s(1:3,:) + J_s(4:6,:);
end