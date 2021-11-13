
%% kinematics

psi=[pi 40 0 0 0 0];
[phi, lf, theta1, delta1, theta2, delta2]=getPsi(psi);
[r_L1, L1, r_Lr, Lr, r_L2, L2, r_Lg, Lg, r_trocar]=getToolArmStructureParameter;

if(lf<0)
    Ttip=eye(4);
end
if(lf<Lg)
    
    Ttip=eye(4);
    Ttip(3,4)=lf;
    S=[0,0;0,0;0,lf;1,1];
end
if(lf<sum([L2 Lg]))
    [l2t,d2t,theta2t]=getArcLength(psi, 2);
    Ttc22b=[
    cos(phi) -sin(phi) 0 -cos(delta1)*d2t;
    sin(phi) cos(phi) 0 -sin(delta1)*d2t;
    0 0 1 -l2t;
    0 0 0 1;
    ];
    
    [T2b22c, s2b22c]=getTib2ie(theta2t, delta2,l2t);
    [T2c22e, s2c22e]=getTib2ie(theta2-theta2t, delta2,L2-l2t);
    T2e2g=eye(4);T2e2g(3,4)=Lg;
    Ttip=Ttc22b*T2b22c*T2c22e*T2e2g;
    s2e2g=[0,0;0,0;0,lf;1,1];
    S=[Ttc22b*s2b22c;Ttc22b*T2b22c*s2c22e;Ttc22b*T2b22c*T2c22e*s2e2g];
end

if(lf<sum([Lr L2 Lg]))
    [l2t,d2t,theta2t]=getArcLength(psi, 2);
    Ttc21e=[
    cos(phi) -sin(phi) 0 -cos(delta1)*d2t;
    sin(phi) cos(phi) 0 -sin(delta1)*d2t;
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
    S=[Ttc21e*s1e22b;Ttc21e*T1e22b*s2b22c;Ttc21e*T1e22b*T2b22c*s2c22e;...
        Ttc21e*T1e22b*T2b22c*T2c22e*s2e2g];
end
if(lf<sum([L1 Lr L2 Lg]))
    [l1t,d1t,theta1t]=getArcLength(psi, 1);
    Ttc21b=[
    cos(phi) -sin(phi) 0 -cos(delta1)*d1t;
    sin(phi) cos(phi) 0 -sin(delta1)*d1t;
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
    S=[Ttc21b*s1b21c;Ttc21b*T1b21c*s1c21e;Ttc21b*T1b21c*T1c21e*s1e22b;...
        Ttc21b*T1b21c*T1c21e*T1e22b*s2b22c;Ttc21b*T1b21c*T1c21e*T1e22b*T2b22c*s2c22e;...
        Ttc21b*T1b21c*T1c21e*T1e22b*T2b22c*T2c22e*s2e2g];
end
if(lf>sum([L1 Lr L2 Lg]))
    [l1t,d1t,theta1t]=getArcLength(psi, 1);
    Ttc21b=[
    cos(phi) -sin(phi) 0 -cos(delta1)*d1t;
    sin(phi) cos(phi) 0 -sin(delta1)*d1t;
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
    S=[Ttc21b*s1b21c;Ttc21b*T1b21c*s1c21e;Ttc21b*T1b21c*T1c21e*s1e22b;...
        Ttc21b*T1b21c*T1c21e*T1e22b*s2b22c;Ttc21b*T1b21c*T1c21e*T1e22b*T2b22c*s2c22e;...
        Ttc21b*T1b21c*T1c21e*T1e22b*T2b22c*T2c22e*s2e2g];
end