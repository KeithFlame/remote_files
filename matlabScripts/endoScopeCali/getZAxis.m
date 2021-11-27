function direction=getZAxis(lineData)
lineData=reshape(lineData,[3 size(lineData,3)]);
lineData=lineData';
x=lineData(:,1);
y=lineData(:,2);
z=lineData(:,3); 
%% 
xyz0(1)=mean(x);
xyz0(2)=mean(y);
xyz0(3)=mean(z);
%% 
centeredLine=bsxfun(@minus,lineData,xyz0);
[U,S,V]=svd(centeredLine);
direction=V(:,1);%方向向量
end


%% 
% % % % % % % % % % lineData=Tcb2st(1:3,4,1:7);
% % % % % % % % % % lineData=reshape(lineData,[3 7]);
% % % % % % % % % % lineData=lineData';
% % % % % % % % % % x=lineData(:,1);
% % % % % % % % % % y=lineData(:,2);
% % % % % % % % % % z=lineData(:,3);
% % % % % % % % % % scatter3(x, y, z,'filled')  %散点图函数，'filled'表示画实心点
% % % % % % % % % % hold on;    %画了一幅图，再画另一幅图时，原来的图还在，与新图共存
% % % % % % % % % % %% 计算平均值（拟合的直线必过所有坐标的算数平均值）
% % % % % % % % % % xyz0(1)=mean(x);
% % % % % % % % % % xyz0(2)=mean(y);
% % % % % % % % % % xyz0(3)=mean(z);%拟合点坐标
% % % % % % % % % % %% 奇异值分解计算方向向量(第一种方法)
% % % % % % % % % % % 协方差矩阵奇异变换
% % % % % % % % % % % 所得直线的方向实际上与最大奇异值对应的奇异向量相同
% % % % % % % % % % centeredLine=bsxfun(@minus,lineData,xyz0);
% % % % % % % % % % [U,S,V]=svd(centeredLine);
% % % % % % % % % % direction2=V(:,1);%方向向量
% % % % % % % % % % 
% % % % % % % % % % %% 最小二乘计算方向向量(第二种方法)
% % % % % % % % % % A=0;
% % % % % % % % % % B=0;
% % % % % % % % % % C=0;
% % % % % % % % % % D=0;
% % % % % % % % % % E=0;
% % % % % % % % % % F=0;
% % % % % % % % % % for i=1:size(lineData,1)
% % % % % % % % % % A=A+x(i)*z(i);
% % % % % % % % % % B=B+x(i);
% % % % % % % % % % C=C+y(i)*z(i);
% % % % % % % % % % D=D+y(i);
% % % % % % % % % % E=E+z(i)*z(i);
% % % % % % % % % % F=F+z(i);
% % % % % % % % % % end
% % % % % % % % % % Tt=[A,B;C,D]*[E,F;F,11]^-1;
% % % % % % % % % % R=sqrt(Tt(1)^2+Tt(2)^2+1);
% % % % % % % % % % direction(1)=Tt(1)/R;
% % % % % % % % % % direction(2)=Tt(2)/R;
% % % % % % % % % % direction(3)=1/R;
% % % % % % % % % % direction=direction';%方向向量
% % % % % % % % % % %% 画图
% % % % % % % % % % 
% % % % % % % % % % direction=direction2;
% % % % % % % % % % t=-100:0.1:100;
% % % % % % % % % % xx=xyz0(1)+direction(1)*t;
% % % % % % % % % % yy=xyz0(2)+direction(2)*t;
% % % % % % % % % % zz=xyz0(3)+direction(3)*t;
% % % % % % % % % % plot3(xx,yy,zz)