function T = fkine(S,M,q,frame)
    % your code here
    T = eye(4);
    size = length(q);
    for i = 1:size
        TI = twist2ht(S(:,i),q(i));
        T = T * TI;
    end
    
    if strcmp(frame,'body')
        T = M*T;
    elseif strcmp(frame,'space')
        T = T*M;
    end
   
    % If needed, you can convert twists to homogeneous transformation matrices with:
    % twist2ht(S(i),q(i));
end