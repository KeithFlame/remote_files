function [n]=tipForceEst(u,MP,oneSegIdx)
if(nargin == 2 || oneSegIdx ==0)
Nm1=39;Nm2=65;
p=zeros(3,Nm2+1);R=eye(3);
m=zeros(3,Nm2);
for i=1:Nm1-1
    K_bt=4*MP.Kb1+4*MP.Kb2;
    ds=u(3,i+1)-u(3,i);
    m(:,i)=R*K_bt*[u(1:2,i);0];
    p_dot=R*[0 0 1]';
    p(:,i+1)=p(:,i)+p_dot*ds;
    dR=S(ds*u(:,i));
    R=R*expm(dR);    
end
R1=R;
p(:,Nm1+1)=R1*[0 0 MP.Lr]'+p(:,Nm1);
m(:,Nm1)=R*K_bt*[u(1:2,Nm1);0];
for i=40:Nm2-1
    K_bt=4*MP.Kb2;
    ds=u(3,i+1)-u(3,i);
    m(:,i)=R*K_bt*[u(1:2,i);0];
    p_dot=R*[0 0 1]';
    p(:,i+1)=p(:,i)+p_dot*ds;
    dR=S(ds*u(:,i));
    R=R*expm(dR);   
end
R2=R;
p(:,Nm2+1)=R2*[0 0 MP.Lg]'+p(:,Nm2);
m(:,Nm2)=R*K_bt*[u(1:2,Nm2);0];

else
Nm2=101;
p=zeros(3,Nm2+1);R=eye(3);
m=zeros(3,Nm2);
for i=1:Nm2-1
    K_bt=4*MP.Kb1;
    ds=u(3,i+1)-u(3,i);
    m(:,i)=R*K_bt*[u(1:2,i);0];
    p_dot=R*[0 0 1]';
    p(:,i+1)=p(:,i)+p_dot*ds;
    dR=S(ds*u(:,i));
    R=R*expm(dR);    
end
R1=R;
p(:,Nm2+1)=R1*[0 0 MP.Lg]'+p(:,Nm2);
m(:,Nm2)=R*K_bt*[u(1:2,Nm2);0];
    
end
%figure(1);hold on;
%plot3(p(1,:),p(2,:),p(3,:),'-b','LineWidth',2);
load('./0925/forcedirect.mat');
figure(1);hold on;
plot3(p(1,:),p(2,:),p(3,:),'-','color',[0.9 0.1 0.3],'LineWidth',2)
plotArrow(p(:,end),vecl(:,3),1.5*4e-2,[1 0 0]);

if(nargin == 2 || oneSegIdx ==0)
j=1;dm=zeros(3,Nm2-1);dp=zeros(3,Nm2-1);
Pd=zeros(3,1);
Pd_m=zeros(3,Nm2-1);
% Pd=zeros(2,1);
% Pd_m=zeros(2,Nm2-1);
for i=1:Nm1-1
    %ds=u(3,i+1)-u(3,i);
    dm(:,j)=(m(:,i+1)-m(:,i));
    dp(:,j)=(p(:,i+1)-p(:,i));
    Pd_m(:,j)=pinv(S(dp(:,j)))*dm(:,j);
    %Pd_m(:,j)=pinv(S(dp(:,j))*[1 0;0 1;0 0])*dm(:,j);
    Pd=Pd+Pd_m(:,j);
    j=j+1;
end
for i=Nm1+1:Nm2-1
    ds=u(3,i+1)-u(3,i);
    dm(:,j)=(m(:,i+1)-m(:,i));
    dp(:,j)=(p(:,i+1)-p(:,i));
    Pd_m(:,j)=pinv(S(dp(:,j)))*dm(:,j);
    %Pd_m(:,j)=pinv(S(dp(:,j))*[1 0;0 1;0 0])*dm(:,j);
    Pd=Pd+Pd_m(:,j);
    j=j+1;
end


Lam_l = zeros(3*(j-1),3);
Lam_r = zeros(3*(j-1),(j-1));

Pd_row=zeros(3*(j-1),1);
for i=1:j-1
    Lam_l(3*(i-1)+1:3*i,:)=eye(3);
    Lam_r(3*(i-1)+1:3*i,i)=-dp(:,i);
    Pd_row(3*(i-1)+1:3*i,1)=-Pd_m(:,i);
end
n_lambda = pinv([Lam_l Lam_r])*Pd_row;
n=n_lambda(1:3,1);

LEFT = zeros(3*(j-1),2);
RIGHT= zeros(3*(j-1),1);
for i=1:j-1
    A=S(dp(:,i))*[1 0;0 1;0 0];
    LEFT(3*(i-1)+1:3*i,:)=-A;
    RIGHT(3*(i-1)+1:3*i,1)=dm(:,i);
end
n2_lambda = pinv(LEFT)*RIGHT;
n2=n2_lambda(1:2,1);

else
    n=[0 0 0]';
end

end