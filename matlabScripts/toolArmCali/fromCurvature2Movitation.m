function qa=fromCurvature2Movitation(u,x,temendoPsi)

if(temendoPsi(2)<x(1))
    L1=temendoPsi(2)+1e-15;
    L0=0;
else
    L1=x(1);
    L0=temendoPsi(2)-x(1);
end

Lr=x(2);
L2=x(3);
Lg=x(4);
gamma=x(5);
K1=x(6);
K2=x(7);
E=50e9;
Lstem=x(9);
sL1=x(1);
qa=zeros(size(u,1),24);


e1=[1 0 0]';e2=[0 1 0]';e3=[0 0 1]';
MP.E=E;                             %Rod Young's modules
MP.mu=0.25;                         %Rod Poisson rate
MP.d1=0.95e-3;                      %Rod diameter seg1
MP.d2=0.4e-3;                       %Rod diameter seg2
MP.d3=0.95e-3;                      %Rod diameter prox actuation
MP.rho1=2.5e-3;                     %Rod pitch circle radius seg1
MP.rho2=2.7e-3;                     %Rod pithc circle radius seg2
MP.rho3=12e-3;                      %prox pitch circle radius (both seg2 and actuation)
MP.alpha = MP.rho3/MP.rho2;
MP.L=Lstem*1e-3;                    %Robot stem length
MP.L1=L1*1e-3;                      %Robot seg1 length
MP.L2=L2*1e-3;                      %Robot seg2 length
MP.L3=18*1e-3;                      %Robot prox length
MP.Lc=200e-3;                       %robot cannula length
MP.Lr=Lr*1e-3;                      %Robot rigid seg length
MP.Lg=Lg*1e-3;                      %Robot gipper length
MP.L0=L0*1e-3;
MP.r11=[1 0 0]'*MP.rho1;MP.r12=[0 1 0]'*MP.rho1;MP.r13=[-1 0 0]'*MP.rho1;MP.r14=[0 -1 0]'*MP.rho1;
MP.r21=[cos(21/180*pi) sin(21/180*pi) 0]'*MP.rho2;
MP.r22=[cos(37/180*pi) sin(37/180*pi) 0]'*MP.rho2;
MP.r23=[cos(53/180*pi) sin(53/180*pi) 0]'*MP.rho2;
MP.r24=[cos(69/180*pi) sin(69/180*pi) 0]'*MP.rho2;
MP.r25=[cos(111/180*pi) sin(111/180*pi) 0]'*MP.rho2;
MP.r26=[cos(127/180*pi) sin(127/180*pi) 0]'*MP.rho2;
MP.r27=[cos(143/180*pi) sin(143/180*pi) 0]'*MP.rho2;
MP.r28=[cos(159/180*pi) sin(159/180*pi) 0]'*MP.rho2;
MP.r29=[cos(201/180*pi) sin(201/180*pi) 0]'*MP.rho2;
MP.r2a=[cos(217/180*pi) sin(217/180*pi) 0]'*MP.rho2;
MP.r2b=[cos(233/180*pi) sin(233/180*pi) 0]'*MP.rho2;
MP.r2c=[cos(249/180*pi) sin(249/180*pi) 0]'*MP.rho2;
MP.r2d=[cos(291/180*pi) sin(291/180*pi) 0]'*MP.rho2;
MP.r2e=[cos(307/180*pi) sin(307/180*pi) 0]'*MP.rho2;
MP.r2f=[cos(323/180*pi) sin(323/180*pi) 0]'*MP.rho2;
MP.r2g=[cos(339/180*pi) sin(339/180*pi) 0]'*MP.rho2;
MP.r31=[1 0 0]'*MP.rho3;MP.r32=[0 1 0]'*MP.rho3;MP.r33=[-1 0 0]'*MP.rho3;MP.r34=[0 -1 0]'*MP.rho3;
MP.Q1=[S(MP.r11)*e3 S(MP.r12)*e3 S(MP.r13)*e3 S(MP.r14)*e3];
MP.Q2=[S(MP.r21)*e3 S(MP.r22)*e3 S(MP.r23)*e3 S(MP.r24)*e3 ...
       S(MP.r25)*e3 S(MP.r26)*e3 S(MP.r27)*e3 S(MP.r28)*e3 ...
       S(MP.r29)*e3 S(MP.r2a)*e3 S(MP.r2b)*e3 S(MP.r2c)*e3 ...
       S(MP.r2d)*e3 S(MP.r2e)*e3 S(MP.r2f)*e3 S(MP.r2g)*e3];
MP.Q3=[S(MP.r31)*e3 S(MP.r32)*e3 S(MP.r33)*e3 S(MP.r34)*e3];
MP.I1=pi*MP.d1^4/64;MP.A1=pi*MP.d1^2/4;
MP.I2=pi*MP.d2^4/64;MP.A2=pi*MP.d2^2/4;
MP.I3=pi*MP.d3^4/64;MP.A3=pi*MP.d3^2/4;
MP.G=MP.E/2/(1+MP.mu);
MP.J1=2*MP.I1;MP.J2=2*MP.I2;MP.J3=2*MP.I3;
MP.Ke1=diag([MP.G*MP.A1 MP.G*MP.A1 MP.A1*MP.E]);
MP.Ke2=diag([MP.G*MP.A2 MP.G*MP.A2 MP.A2*MP.E]);
MP.Ke3=diag([MP.G*MP.A3 MP.G*MP.A3 MP.A3*MP.E]);
MP.Kb1=diag([MP.E*MP.I1 MP.E*MP.I1 2*MP.G*MP.I1]);
MP.Kb2=diag([MP.E*MP.I2 MP.E*MP.I2 2*MP.G*MP.I2]);
MP.Kb3=diag([MP.E*MP.I3 MP.E*MP.I3 2*MP.G*MP.I3]);
MP.gamma =gamma;

dGamma = zeros(24,9);
Ell=zeros(24,24);
Theta=zeros(9,24);
K=zeros(9,9);

dGamma(1:4,1:3)=(MP.L1+MP.L0*MP.gamma)*MP.Q1';
dGamma(5:20,1:3)=(MP.L1+MP.L0*MP.gamma)*MP.Q2';
dGamma(5:20,4:6)=MP.L2*MP.Q2';
dGamma(5:20,7:9)=-MP.alpha*MP.L3*MP.Q2';
dGamma(21:24,7:9)=-MP.L3*MP.Q3';

Ell(1:4,1:4)=diag([1 1 1 1])*(sL1*1e-3+MP.L);%%%%
Ell(5:20,5:20)=diag([1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1])*(MP.L2+MP.Lr+sL1*1e-3+MP.L+MP.Lc+MP.L3);
Ell(21:24,21:24)=diag([1 1 1 1])*MP.L3;
    
Theta(1:3,1:4)=-MP.A1*MP.E*MP.Q1;
Theta(4:6,5:20)=-MP.A2*MP.E*MP.Q2;
Theta(7:9,21:24)=-MP.A3*MP.E*MP.Q3;

K(1:3,1:3)=4*MP.Kb1+16*MP.Kb2+K1*MP.Kb1;
K(1:3,4:6)=-16*MP.Kb2-K2*MP.Kb1;
K(4:6,4:6)=16*MP.Kb2+K2*MP.Kb1;
K(7:9,4:6)=MP.alpha*(16*MP.Kb2+K2*MP.Kb1);
K(7:9,7:9)=4*MP.Kb3+16*MP.Kb2;



G=dGamma-Ell*pinv(Theta)*K;



for i = 1:size(u,1)
    uc=u(i,:);
    u1=uc(1:3)';u2=uc(4:6)';
    u3=pinv(MP.Q2')/MP.alpha/MP.L3*( MP.Q2'*(MP.L1+MP.gamma*MP.L0)*u1+...
        (MP.Q2'*MP.L2+ (MP.L2+MP.Lr+x(1)*1e-3+MP.L+MP.Lc+MP.L3)/MP.A2/MP.E*pinv(MP.Q2)*...   %x(1)*1e-3
        (16*MP.Kb2+K2*MP.Kb1))*u2 );
    ucc=[u1;u2;u3];
    q=G*ucc;
    qa(i,:)=q';
end
end