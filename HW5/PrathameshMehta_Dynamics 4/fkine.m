function T = fkine(S, M, q, frame)
    T = eye(4);
    
    if strcmp(frame, 'space')
        for i = 1:length(q)
            T = T * twist2ht(S(:,i), q(i));
        end
    elseif strcmp(frame, 'body')
        for i = length(q):-1:1
            T = twist2ht(S(:,i), q(i)) * T;
        end
        T = M * T;
    else
        error('Invalid frame type. Choose either "space" or "body".');
    end
    
    if strcmp(frame, 'space')
        T = T * M;
    end
end
