function T = fkine(S,M,q)
     t = eye(4);
    % If needed, you can convert twists to homogeneous transformation matrices with:
    for i = 1:width(K)  %for each screw axis calculating the transfomation matrix 
        t = t*twist2ht(K(1:end,i),q(i)); %multiplying with previous tranfomation matrix
    end
    T = t*M; %multiplying with home configuration 
end