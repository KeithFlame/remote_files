function df = discreteFrechetDistance(P, Q)
    % P 和 Q 是 n×2 或 m×2 的矩阵，表示两条线上的坐标序列
    
    function D = computeDMatrix(P,Q)
        nP = size(P, 1);
        nQ = size(Q, 1);
        
        D = zeros(nP,nQ);

        for i=1:nP
            for j=1:nQ
                d(i,j)=norm(P(i,:)-Q(j,:),2); 
            end
        end
        
        ca=zeros(nP,nQ);
    
        ca(1,1) = d(1,1);
        for i=2:nP
            ca(i,1)=max([ca(i-1,1),d(i,1)]);
        end
        for j=2:nQ
            ca(1,j)=max([ca(1,j-1),d(1,j)]);  
        end

        for i=2:nP
            for j=2:nQ
                ca(i,j)=max(min([ca(i-1,j),ca(i,j-1),ca(i-1,j-1)]),d(i,j));
            end
        end   
        D = ca;
    end
    
    D_matrix = computeDMatrix(P,Q);
    df = D_matrix(end,end);
end
