function yDirection=getYAxis(planeData)

planeData=reshape(planeData,[3 size(planeData,3)]);
planeData=planeData';

% Э��������SVD�任�У���С����ֵ��Ӧ��������������ƽ��ķ���
xyz0=mean(planeData,1);
centeredPlane=bsxfun(@minus,planeData,[0 0 0]);
[~,~,V]=svd(centeredPlane);
 
yDirection=V(:,3);
end