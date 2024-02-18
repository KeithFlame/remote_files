function planes = fitPlane_SVD(data)
% 功能：利用SVD拟合平面
% 输入：data   - 原始数据(m*3)    
% 输出：planes - 拟合所得平面参数 
planes = zeros(4,1);
points = data(:,1:3);
% 去质心
M = points - repmat(mean(points),size(points,1),1);
% 奇异值分解
[~,~,V] = svd(M,0);
% 最小特征值对应的特征向量为法向量
normal = V(:,3)';
% 平面参数标准化
dtmp = mean(points*normal');
planes(1:3) = normal'*sign(dtmp);
planes(1:3) = planes(1:3)/norm(planes(1:3));
planes(4) = -dtmp*sign(dtmp);
if(planes(3)<0)
    planes = -planes;
end
end