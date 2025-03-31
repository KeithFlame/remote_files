function [P_proj,N] = projectPointToPlane(P,A,U,V)

% 计算法向量 N
N = cross(U, V);
N = N / norm(N);

% 计算 AP 向量
AP = P - A;

% 计算投影
proj_N_AP = (dot(AP, N) / dot(N, N)) * N;

% 计算投影点
P_proj = P - proj_N_AP;

end