Ls=220;
L1=100;
Lr=10;
L2=20;
Lg=15;

theta1_lim=pi/2;
theta2_lim=pi*2/3;

t= 1; % 臂体与鞘管之间的单侧间隙
scaler=0:0.1:1;
%% 正运动学
%      L  phi theta1 delta1 theta2 delta2
psi = [40   0    0      0      0      0];

l=psi(1);phi=psi(2);theta1=psi(3);delta1=psi(4);theta2=psi(5);delta2=psi(6);

T1=eye(4);  % phi Ls
T2=eye(4);  % 鞘管内部的L1部分
T3=eye(4);  % 鞘管外部的L1部分
T4=eye(4);  % Lr
T5=eye(5);  % L2
T6=eye(6);  % Lg
if(l<0)
    

elseif(l<Lg)
    T6(3,4)=l;  % Lg
    s6=[0 0;0 0;0 l];
elseif(l<Lg+L2)
    l2=l-Lg;
    k2=theta2/l2;
    cosT2=cos(theta2);sinT2=sin(theta2);cosD2=cos(delta2);sinD2=sin(delta2);
    T5=[(cosD2)^2*(cosT2-1)+1 sinD2*cosD2*(cosT2-1) cosD2*sinT2 cosD2*(1-cosT2)/k2
        sinD2*cosD2*(cosT2-1) (cosD2)^2*(1-cosT2)+cosT2 sinD2*sinT2 sinD2*(1-cosT2)/k2
        -cosD2*sinT2 -sinD2*sinT2 cosT2 sinT2/k2
        0 0 0 1];
    s5=[cosD2*(1-cos(scaler*THETA2))/k2;
        sinD2*(1-cos(scaler*THETA2))/k2;
        sin(scaler*THETA2)/k2;
        ones(1,length(scaler))];%Seg2 
    T6(3,4)=Lg;
    s6=[0 0;0 0;0 Lg];
end


