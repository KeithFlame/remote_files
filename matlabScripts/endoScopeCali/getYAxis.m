function yDirection=getYAxis(planeData)

planeData=reshape(planeData,[3 size(planeData,3)]);
planeData=planeData';

% 协方差矩阵的SVD变换中，最小奇异值对应的奇异向量就是平面的方向
xyz0=mean(planeData,1);
centeredPlane=bsxfun(@minus,planeData,[0 0 0]);
[~,~,V]=svd(centeredPlane);
 
yDirection=V(:,3);
end