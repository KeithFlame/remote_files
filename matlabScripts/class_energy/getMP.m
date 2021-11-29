function MP=getMP(L0)

if(nargin==0)
    L0=0;
end

[~, L1, ~, Lr, ~, L2, ~, Lg,~,Lstem,zeta]=getToolArmStructureParameter;


E=50e9;
e3=[0 0 1]';
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
MP.L0=L0*1e-3;                      %Robot 
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
MP.zeta =zeta;
end