function [direction,origin] = getZ(Lg, start, P)
%% 计算平均值（拟合的直线必过所有坐标的算数平均值）
xyz0(1)=mean(P(:,1));
xyz0(2)=mean(P(:,2));
xyz0(3)=mean(P(:,3));%拟合点坐标
%% 奇异值分解计算方向向量(第一种方法)
% 协方差矩阵奇异变换
% 所得直线的方向实际上与最大奇异值对应的奇异向量相同
centeredLine=bsxfun(@minus,P,xyz0);
[U,S,V]=svd(centeredLine);
direction=V(:,1);%方向向量
if(direction(2)>0)
    direction= -direction;
end
P2=P;
for i = 1:size(P,1)
    % P2(i,:)=P(i,:)-direction'*(45+20+10+19.4+5*i);
        P2(i,:)=P(i,:)-direction'*(start+10+19.4+Lg+5*(i-1));

end

origin = mean(P2);

end

