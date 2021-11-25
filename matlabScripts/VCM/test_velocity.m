clc
clear
L1=0.04;
L2=0.06;
L=L1+L2;
R1=0.1;
R2=0.05;
theta1=L1/R1;
theta2=L2/R2;
r=3e-3;
q1=theta1*r;
q2=theta2*r;
q=q1+q2;
theta = q/r;
R=L/theta;

global Posi_1 Ori_1;
Posi_1=zeros(3,1);
Ori_1=eye(3);
PlotSnake(0,theta1,L1,[0.003 1000])
grid on
axis equal
PlotSnake(0,theta2,L2,[0.003 1000])

Posi_1=zeros(3,1);
Ori_1=eye(3);
PlotSnake(0,theta,L,[0.003 1000])
axis([-0.01 0.08 -0.03 0.03 -0.01 0.1])

% for i=1:40
%     global Posi_1 Ori_1;
%     Posi_1=zeros(3,1);
%     Ori_1=eye(3);
%     PlotSnake(0,theta1+0.24/40*i,L1,[0.003 1000])
%     grid on
%     axis equal
%     PlotSnake(0,theta2-0.24/40*i,L2,[0.003 1000])
%     pause(0.1)
% end



N=100;
Rot=eye(3);p=zeros(3,N);velocity=zeros(3,N);
du1=1/R-1/R1;
du2=1/R-1/R2;
e3=[0 0 1]';

for i=1:N-1
    v=L/N;
    if(i<=L1/L*N)
        U(:,i)=v*[0 1/R1 0]';
        du=v*[0 du1 0]';
    else
        U(:,i)=v*[0 1/R2 0]';
        du=v*[0 du2 0]';
    end
    dR=S(U(:,i));
    dp=Rot*e3;
    
    velocity_dot = v*Rot*S(du)*e3;
    velocity(:,i+1)=velocity(:,i)+velocity_dot;
    
    Rot=Rot*expm(dR);
    
    p(:,i+1)=p(:,i)+v*dp;
    
end
%plot3(p(1,:),p(2,:),p(3,:),'*b');

for i=1:N
    plot3([p(1,i) p(1,i)+velocity(1,i)*50],[p(2,i) p(2,i)+velocity(2,i)*50],[p(3,i) p(3,i)+velocity(3,i)*50],'-b');
end

