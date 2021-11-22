
%% kinematics

psi=[pi 30 0 0 pi/2 pi/3];
[phi, lf, theta1, delta1, theta2, delta2]=getPsi(psi);
[r_L1, L1, r_Lr, Lr, r_L2, L2, r_Lg, Lg, r_trocar]=getToolArmStructureParameter;

Config_plot=zeros(2,7);
T_config=zeros(4,4,7);
for i =1:size(T_config,3)
    T_config(:,:,i)=eye(4);
end

if(lf<0)
    Ttip=eye(4);
    S0=[0,0;0,0;0,0;1,1];
    S1=[0,0;0,0;0,0;1,1];
    S2=[0,0;0,0;0,0;1,1];
    S3=[0,0;0,0;0,0;1,1];
    S4=[0,0;0,0;0,0;1,1];
    S5=[0,0;0,0;0,0;1,1];
    S6=[0,0;0,0;0,0;1,1];
elseif(lf<Lg)
    Ttip=eye(4);
    Ttip(3,4)=lf;
    S0=[0,0;0,0;0,0;1,1];
    S1=[0,0;0,0;0,0;1,1];
    S2=[0,0;0,0;0,0;1,1];
    S3=[0,0;0,0;0,0;1,1];
    S4=[0,0;0,0;0,0;1,1];
    S5=[0,0;0,0;0,0;1,1];
    S6=[0,0;0,0;0,lf;1,1];
    Config_plot(:,7)=[lf,0];
    T_config(:,:,7)=eye(4);
elseif(lf<sum([L2 Lg]))
    [l2t,d2t,theta2t]=getArcLength(psi, 2);
    Ttc22b=[
    cos(phi) -sin(phi) 0 -cos(delta2+phi)*d2t;
    sin(phi) cos(phi) 0 -sin(delta2+phi)*d2t;
    0 0 1 lf-sum([L2 Lg]);
    0 0 0 1;
    ];
    
    [T2b22c, s2b22c]=getTib2ie(theta2t, delta2,l2t);
    [T2c22e, s2c22e]=getTib2ie(theta2-theta2t, delta2,L2-l2t);
    T2e2g=eye(4);T2e2g(3,4)=Lg;
    Ttip=Ttc22b*T2b22c*T2c22e*T2e2g;
    s2e2g=[0,0;0,0;0,Lg;1,1];

    S0=Ttc22b*s2b22c;
    S1=[0,0;0,0;0,0;1,1];
    S2=[0,0;0,0;0,0;1,1];
    S3=[0,0;0,0;0,0;1,1];
    S4=[0,0;0,0;0,0;1,1];
    S5=Ttc22b*T2b22c*s2c22e;
    S6=Ttc22b*T2b22c*T2c22e*s2e2g;

    Config_plot(:,5)=[l2t,theta2t];
    Config_plot(:,6)=[(L2-l2t),theta2-theta2t];
    Config_plot(:,7)=[Lg,0];
    if(theta2t==0)
        Config_plot(:,5)=[l2t,0];
    end
    if(theta2==0)
        Config_plot(:,6)=[(L2-l2t),0];
    end
    T_config(:,:,5)=Ttc22b;
    T_config(:,:,6)=Ttc22b*T2b22c;
    T_config(:,:,7)=Ttc22b*T2b22c*T2c22e;
elseif(lf<sum([Lr L2 Lg]))
    [l2t,d2t,theta2t]=getArcLength(psi, 2);
    Ttc21e=[
    cos(phi) -sin(phi) 0 -cos(delta2)*d2t;
    sin(phi) cos(phi) 0 -sin(delta2)*d2t;
    0 0 1 lf-sum([Lr L2 Lg]);
    0 0 0 1;
    ];
    T1e22b=eye(4);T1e22b(3,4)=Lr;
    s1e22b=[0,0;0,0;0,Lr;1,1];
    [T2b22c, s2b22c]=getTib2ie(theta2t, delta2,l2t);
    [T2c22e, s2c22e]=getTib2ie(theta2-theta2t, delta2,L2-l2t);
    T2e2g=eye(4);T2e2g(3,4)=Lg;
    Ttip=Ttc21e*T1e22b*T2b22c*T2c22e*T2e2g;
    s2e2g=[0,0;0,0;0,Lg;1,1];
    
    S0=[0,0;0,0;0,0;1,1];
    S1=[0,0;0,0;0,0;1,1];
    S2=[0,0;0,0;0,0;1,1];
    S3=Ttc21e*s1e22b;
    S4=Ttc21e*T1e22b*s2b22c;
    S5=Ttc21e*T1e22b*T2b22c*s2c22e;
    S6=Ttc21e*T1e22b*T2b22c*T2c22e*s2e2g;

    Config_plot(:,4)=[Lr,0];
    Config_plot(:,5)=[l2t,theta2t];
    Config_plot(:,6)=[(L2-l2t),theta2-theta2t];
    Config_plot(:,7)=[Lg,0];
    if(theta2t==0)
        Config_plot(:,5)=[l2t,0];
    end
    if(theta2==0)
        Config_plot(:,6)=[(L2-l2t),0];
    end

    T_config(:,:,4)=Ttc21e;
    T_config(:,:,5)=Ttc21e*T1e22b;
    T_config(:,:,6)=Ttc21e*T1e22b*T2b22c;
    T_config(:,:,7)=Ttc21e*T1e22b*T2b22c*T2c22e;
elseif(lf<sum([L1 Lr L2 Lg]))
    [l1t,d1t,theta1t]=getArcLength(psi, 1);
    Ttc21b=[
    cos(phi) -sin(phi) 0 -cos(delta1+phi)*d1t;
    sin(phi) cos(phi) 0 -sin(delta1+phi)*d1t;
    0 0 1 lf-sum([Lr L2 Lg]);
    0 0 0 1;
    ];
    
    [T1b21c, s1b21c]=getTib2ie(theta1t, delta1,l1t);
    [T1c21e, s1c21e]=getTib2ie(theta1-theta1t, delta1,L1-l1t);

    T1e22b=eye(4);T1e22b(3,4)=Lr;
    s1e22b=[0,0;0,0;0,Lr;1,1];
    [l2t,d2t,theta2t]=getArcLength(psi, 2);

    
    [T2b22c, s2b22c]=getTib2ie(theta2t, delta2,l2t);
    [T2c22e, s2c22e]=getTib2ie(theta2-theta2t, delta2,L2-l2t);
    T2e2g=eye(4);T2e2g(3,4)=Lg;
    Ttip=Ttc21b*T1b21c*T1c21e*T2b22c*T2c22e*T2e2g;
    s2e2g=[0,0;0,0;0,Lg;1,1];
    

    S0=Ttc21b*s1b21c;
    S1=[0,0;0,0;0,0;1,1];
    S2=Ttc21b*T1b21c*s1c21e;
    S3=Ttc21b*T1b21c*T1c21e*s1e22b;
    S4=Ttc21b*T1b21c*T1c21e*T1e22b*s2b22c;
    S5=Ttc21b*T1b21c*T1c21e*T1e22b*T2b22c*s2c22e;
    S6=Ttc21b*T1b21c*T1c21e*T1e22b*T2b22c*T2c22e*s2e2g;

    Config_plot(:,2)=[l1t,theta1t];
    Config_plot(:,3)=[(L1-l1t),theta1-theta1t];
    Config_plot(:,4)=[lf-sum([L2 Lg]),0];
    Config_plot(:,5)=[l2t,theta2t];
    Config_plot(:,6)=[(L2-l2t),theta2-theta2t];
    Config_plot(:,7)=[Lg,0];
    if(theta1t==0)
        Config_plot(:,5)=[l1t,0];
    end
    if(theta1==0)
        Config_plot(:,6)=[(L1-l1t),0];
    end
    if(theta2t==0)
        Config_plot(:,5)=[l2t,0];
    end
    if(theta2==0)
        Config_plot(:,6)=[(L2-l2t),0];
    end

    T_config(:,:,2)=Ttc21b;
    T_config(:,:,3)=Ttc21b*T1b21c;
    T_config(:,:,4)=Ttc21b*T1b21c*T1c21e;
    T_config(:,:,5)=Ttc21b*T1b21c*T1c21e*T1e22b;
    T_config(:,:,6)=Ttc21b*T1b21c*T1c21e*T1e22b*T2b22c;
    T_config(:,:,7)=Ttc21b*T1b21c*T1c21e*T1e22b*T2b22c*T2c22e;    
elseif(lf>sum([L1 Lr L2 Lg]))
    [l1t,d1t,theta1t]=getArcLength(psi, 1);
    Ttc21b=[
    cos(phi) -sin(phi) 0 -cos(delta1+phi)*d1t;
    sin(phi) cos(phi) 0 -sin(delta1+phi)*d1t;
    0 0 1 lf-sum([L1 Lr L2 Lg]);
    0 0 0 1;
    ];
    
    [T1b21c, s1b21c]=getTib2ie(theta1t, delta1,l1t);
    [T1c21e, s1c21e]=getTib2ie(theta1-theta1t, delta1,L1-l1t);

    T1e22b=eye(4);T1e22b(3,4)=Lr;
    s1e22b=[0,0;0,0;0,Lr;1,1];
    [l2t,d2t,theta2t]=getArcLength(psi, 2);

    
    [T2b22c, s2b22c]=getTib2ie(theta2t, delta2,l2t);
    [T2c22e, s2c22e]=getTib2ie(theta2-theta2t, delta2,L2-l2t);
    T2e2g=eye(4);T2e2g(3,4)=Lg;
    Ttip=Ttc21b*T1b21c*T1c21e*T2b22c*T2c22e*T2e2g;
    s2e2g=[0,0;0,0;0,Lg;1,1];


    S0=Ttc21b*s1b21c;
    S1=[0,0;0,0;0,0;1,1];
    S2=Ttc21b*T1b21c*s1c21e;
    S3=Ttc21b*T1b21c*T1c21e*s1e22b;
    S4=Ttc21b*T1b21c*T1c21e*T1e22b*s2b22c;
    S5=Ttc21b*T1b21c*T1c21e*T1e22b*T2b22c*s2c22e;
    S6=Ttc21b*T1b21c*T1c21e*T1e22b*T2b22c*T2c22e*s2e2g;

    Config_plot(:,1)=[lf-sum([L1 Lr L2 Lg]),0];
    Config_plot(:,2)=[l1t,theta1t];
    Config_plot(:,3)=[(L1-l1t),theta1-theta1t];
    Config_plot(:,4)=[lf-sum([L2 Lg]),0];
    Config_plot(:,5)=[l2t,theta2t];
    Config_plot(:,6)=[(L2-l2t),theta2-theta2t];
    Config_plot(:,7)=[Lg,0];
    if(theta1t==0)
        Config_plot(:,5)=[l1t,0];
    end
    if(theta1==0)
        Config_plot(:,6)=[(L1-l1t),0];
    end
    if(theta2t==0)
        Config_plot(:,5)=[l2t,0];
    end
    if(theta2==0)
        Config_plot(:,6)=[(L2-l2t),0];
    end
    T_config(:,:,1)=Ttc21b;
    T_config(:,:,2)=Ttc21b;
    T_config(:,:,3)=Ttc21b*T1b21c;
    T_config(:,:,4)=Ttc21b*T1b21c*T1c21e;
    T_config(:,:,5)=Ttc21b*T1b21c*T1c21e*T1e22b;
    T_config(:,:,6)=Ttc21b*T1b21c*T1c21e*T1e22b*T2b22c;
    T_config(:,:,7)=Ttc21b*T1b21c*T1c21e*T1e22b*T2b22c*T2c22e; 
end

figure;hold on;grid on; axis equal;

plotSnake(S0,S1,S2,S3,S4,S5,S6,Config_plot, T_config);
% plot3(S0(1,:), S0(2,:), S0(3,:),'b',LineWidth=2);
% plot3(S1(1,:), S1(2,:), S1(3,:),'c',LineWidth=2);
% plot3(S2(1,:), S2(2,:), S2(3,:),'r',LineWidth=2);
% plot3(S3(1,:), S3(2,:), S3(3,:),'g',LineWidth=2);
% plot3(S4(1,:), S4(2,:), S4(3,:),'b',LineWidth=2);
% plot3(S5(1,:), S5(2,:), S5(3,:),'c',LineWidth=2);
% plot3(S6(1,:), S6(2,:), S6(3,:),'r',LineWidth=2);
xlabel('x');ylabel('y');zlabel('z');
% plotTrocar;
