%read and calc TP arm tip positions for streaming
clc;clear;
%please use 2 items: 'position in ref' and 'rotation matrix' in Tracker recording

% reference positions
for i=1:151
    pg(:,i)=[(i-1)*0.2e-3 0 0.09]';
end
for i=152:301
    pg(:,i)=[30e-3 (i-151)*0.2e-3 0.09]';
end
for i=302:601
    pg(:,i)=[0.03-(i-301)*0.2e-3 30e-3 0.09]';
end
for i=602:751
    pg(:,i)=[-0.03 0.03-(i-601)*0.2e-3 0.09]';
end
for i=752:901
    pg(:,i)=[-0.03+(i-751)*0.2e-3 0 0.09]';
end
% for i=1:151
%     pg(:,i)=[(i-1)*0.2e-3 -(i-1)*0.1e-3 0.09]';
% end
% for i=152:301
%     pg(:,i)=[0.03+(i-151)*0.08e-3 -0.015+(i-151)*0.2e-3 0.09]';
% end
% for i=302:451
%     pg(:,i)=[0.042-(i-301)*0.28e-3 0.015+(i-301)*0.1e-3 0.09]';
% end
% for i=452:601
%     pg(:,i)=[0-(i-451)*0.28e-3 0.03-(i-451)*0.1e-3 0.09]';
% end
% for i=602:751
%     pg(:,i)=[-0.042+(i-601)*0.08e-3 0.015-(i-601)*0.2e-3 0.09]';
% end
% for i=752:901
%     pg(:,i)=[-0.03+(i-751)*0.2e-3 -0.015+(i-751)*0.1e-3 0.09]';
% end

% measured positions
data1=importdata('unloadedTracking.txt');
data2=data1.data;
N=length(data2);
R_bC=zeros(3,3,N);
R_tC=zeros(3,3,N);
p_tb=zeros(3,N);
p_TB=zeros(3,N);

%marker geometry and facet coordinate definition
Lb=23;Lt=18;
R_bB=[0 -1 0;-1 0 0;0 0 -1]';p_bB=[12 12+Lb/2 0]';
p_Tt=[-12-Lt/2 12 0]';R_Tt=R_bB';

%extract rule according to data arrangement 
for i=1:N
    R_bC(:,:,i)=[data2(i,5:7);data2(i,8:10);data2(i,11:13)];
    p_tb(:,i)=data2(i,14:16)';
    R_tC(:,:,i)=[data2(i,17:19);data2(i,20:22);data2(i,23:25)];
    %for 1-100
    p_TB(:,i)=p_bB+ R_bB*p_tb(:,i)+R_bB*R_bC(:,:,i)'*R_tC(:,:,i)*p_Tt;
    p_TB(:,i)=p_TB(:,i)-[0 57 54]';
    %for 101-150
    R_TB(:,:,i)=R_bB*R_bC(:,:,i)'*R_tC(:,:,i)*R_Tt;
    theta_y(i) = -asin(R_TB(3,1,i));
    theta_x(i) = atan2(R_TB(3,2,i)/cos(theta_y(i)),R_TB(3,3,i)/cos(theta_y(i)));
    theta_z(i) = atan2(R_TB(2,1,i)/cos(theta_y(i)),R_TB(1,1,i)/cos(theta_y(i)));
    %p_TB(:,i)=p_bB+ R_bB*R_bC(:,:,i)'*(p_tb(:,i)-data2(i,2:4)')+R_bB*R_bC(:,:,i)'*R_tC(:,:,i)*p_Tt;
end
figure(2);hold on;
theta_y=theta_y-theta_y(1);
theta_z=theta_z-theta_z(1);
theta_x=theta_x-theta_x(1);
plot(theta_x,'--r');
plot(theta_y,'--g');
plot(theta_z,'--b');
p_TB=Expm([0 0 0.15]')*p_TB;
p_TB(2,:)=p_TB(2,:)*1.0;
p_TB=p_TB*1e-3;
figure(1);hold on;
plot3(p_TB(1,:),p_TB(2,:),p_TB(3,:),'-.b');
plot3(pg(1,:),pg(2,:),pg(3,:),'-k');

grid on
axis equal
axis([-35 35 -5 60 40 100]*1e-3);
