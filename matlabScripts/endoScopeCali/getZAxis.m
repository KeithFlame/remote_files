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
direction=V(:,1);%��������
end


%% 
% % % % % % % % % % lineData=Tcb2st(1:3,4,1:7);
% % % % % % % % % % lineData=reshape(lineData,[3 7]);
% % % % % % % % % % lineData=lineData';
% % % % % % % % % % x=lineData(:,1);
% % % % % % % % % % y=lineData(:,2);
% % % % % % % % % % z=lineData(:,3);
% % % % % % % % % % scatter3(x, y, z,'filled')  %ɢ��ͼ������'filled'��ʾ��ʵ�ĵ�
% % % % % % % % % % hold on;    %����һ��ͼ���ٻ���һ��ͼʱ��ԭ����ͼ���ڣ�����ͼ����
% % % % % % % % % % %% ����ƽ��ֵ����ϵ�ֱ�߱ع��������������ƽ��ֵ��
% % % % % % % % % % xyz0(1)=mean(x);
% % % % % % % % % % xyz0(2)=mean(y);
% % % % % % % % % % xyz0(3)=mean(z);%��ϵ�����
% % % % % % % % % % %% ����ֵ�ֽ���㷽������(��һ�ַ���)
% % % % % % % % % % % Э�����������任
% % % % % % % % % % % ����ֱ�ߵķ���ʵ�������������ֵ��Ӧ������������ͬ
% % % % % % % % % % centeredLine=bsxfun(@minus,lineData,xyz0);
% % % % % % % % % % [U,S,V]=svd(centeredLine);
% % % % % % % % % % direction2=V(:,1);%��������
% % % % % % % % % % 
% % % % % % % % % % %% ��С���˼��㷽������(�ڶ��ַ���)
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
% % % % % % % % % % direction=direction';%��������
% % % % % % % % % % %% ��ͼ
% % % % % % % % % % 
% % % % % % % % % % direction=direction2;
% % % % % % % % % % t=-100:0.1:100;
% % % % % % % % % % xx=xyz0(1)+direction(1)*t;
% % % % % % % % % % yy=xyz0(2)+direction(2)*t;
% % % % % % % % % % zz=xyz0(3)+direction(3)*t;
% % % % % % % % % % plot3(xx,yy,zz)