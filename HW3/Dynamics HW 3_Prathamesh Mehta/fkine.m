function T = fkine(S,M,q)
     t = eye(4);
    % If needed, you can convert twists to homogeneous transformation matrices with:
    for i = 1:width(S)  %for each screw axis calculating the transfomation matrix 
        t = t*twist2ht(S(1:end,i),q(i)); %multiplying with previous tranfomation matrix
    end
    T = t*M; %multiplying with home configuration 
end