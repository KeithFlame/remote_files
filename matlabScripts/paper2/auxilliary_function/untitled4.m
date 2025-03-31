
% 示例使用
% P = [0, 0, 0; 1, 1, 1; 2, 2, 2];  % 曲线A的点
% Q = [0, 0, 0; 1, 2, 1; 2, 3, 3];  % 曲线B的点
%% Experiment

Q0 = pp_m(1:31,:);
Q1 = pp_m(32:135,:);
Q2 = pp_m(136:265,:);
Q3 = pp_m(266:356,:);
Q4 = pp_m(357:454,:);
block_size0 = size(Q0,1);
block_size1 = size(Q1,1);
block_size2 = size(Q2,1);
block_size3 = size(Q3,1);
block_size4 = size(Q4,1);
P0 = discretizeLineSegment(pp_m(1,:),p_block(1,:),block_size0);
P1 = discretizeLineSegment(p_block(1,:),p_block(2,:),block_size1);
P2 = discretizeLineSegment(p_block(2,:),p_block(3,:),block_size2);
P3 = discretizeLineSegment(p_block(3,:),p_block(4,:),block_size3);
P4 = discretizeLineSegment(p_block(4,:),p_block(5,:),block_size4);

distance0 = computeFrechetDistance3D(P1, Q1);
distance1 = computeFrechetDistance3D(P1, Q1);
distance2 = computeFrechetDistance3D(P2, Q2);
distance3 = computeFrechetDistance3D(P3, Q3);
distance4 = computeFrechetDistance3D(P4, Q4);

fprintf('Frechet Distance1: %.4f\n', distance0);
fprintf('Frechet Distance1: %.4f\n', distance1);
fprintf('Frechet Distance2: %.4f\n', distance2);
fprintf('Frechet Distance3: %.4f\n', distance3);
fprintf('Frechet Distance4: %.4f\n', distance4);

function frechetDistance = computeFrechetDistance3D(P, Q)
    % P 和 Q 是两个空间曲线的点集合，分别为 Nx3 和 Mx3 的矩阵
    N = size(P, 1);
    M = size(Q, 1);
    
    % 创建动态规划表
    dp = inf(N, M);
    
    % 初始化起点
    dp(1, 1) = norm(P(1, :) - Q(1, :));
    
    % 填充动态规划表
    for i = 1:N
        for j = 1:M
            if i == 1 && j == 1
                continue;
            end
            
            if i > 1
                dp(i, j) = min(dp(i, j), dp(i-1, j) + norm(P(i, :) - Q(j, :)));
            end
            
            if j > 1
                dp(i, j) = min(dp(i, j), dp(i, j-1) + norm(P(i, :) - Q(j, :)));
            end
            
            if i > 1 && j > 1
                dp(i, j) = min(dp(i, j), dp(i-1, j-1) + norm(P(i, :) - Q(j, :)));
            end
        end
    end
    
    % Frechet距离为动态规划表右下角的值
    frechetDistance = dp(N, M);
end

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
