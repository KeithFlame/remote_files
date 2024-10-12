function [planes] = getY(P)
%GETX 此处显示有关此函数的摘要
%   此处显示详细说明
points = P(:,1:3);
% 去质心
M = points - repmat(mean(points),size(points,1),1);
% 奇异值分解
[~,~,V] = svd(M,0);
% 最小特征值对应的特征向量为法向量
normal = V(:,3)';
% 平面参数标准化
dtmp = mean(points*normal');
planes(1:3) = normal'*sign(dtmp);
if(planes(1)<0) 
    planes(1:3)=-planes(1:3);
end
planes(4) = -dtmp*sign(dtmp);
end

