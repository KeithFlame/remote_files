function [psi_inv, flag_psi] = IKco_2segs_bendng_keith(T, SL, FM, MBP, discrete_element)
% Declaration
% the end-effector coordinate {g}.
% the base coordinate {b}
%
% This is a function to calculate the inverse kinematics for a 2-seg continuum
% manipulator with the Cosserat rod theory using the approach in:
% Y. Chen, B. Wu, J. Jin, and K. Xu, "A Variable Curvature Model for Multi-
% Backbone Continuum Robots to Account for Inter-Segment Coupling and 
% External Disturbance," IEEE Robotics and Automation Letters, vol. 6, 
% no. 2, pp. 1590-1597, Apr. 2021.
%
% input1: T (4 X 4 Matrix, the pose of the continuum manipulater end)
% input2: structural length SL (4 X 1 vector, L1 Lr L2 Lg)
%                             or (8 X 1 vector additional gamma1 gamma2 gamma3 zeta)
% input3: FM (external concentrated/distributed force/moment, 12 X 1 vector)
% input4: multi-backbone manipulator parameter MBP
% input5: discrete_element (the size of the smallest element of integration, 
% also used in plotting)
% 
% output1: psi (6 X 1 vector, phi L theta1 delta1 theta2 delta2)
% output2: flag_psi (limited-solution: false -1, no-solution: false 0, 
%                    C3-solution: true 1, C4-solution: true 2)
% 
% 
% Author Keith W.
% Ver. 1.0
% Date: 09.02.2023
if(nargin == 1)
    SL = [100 10 20 15]';
    FM = zeros(12,1);
    
    MBP.E=55e9;%Rod Young's modules
    MBP.mu=0.33;%Rod Poisson rate
    MBP.d1=0.95e-3;%Rod diameters
    MBP.d2=0.40e-3;
    MBP.rho1=2.5e-3;%Rod pitch circle radius
    MBP.rho2=2.7e-3;%Rod pitch circle radius
    MBP.L=600e-3;%Robot stem length
    MBP.L1=SL(1)*1e-3;%Robot seg1 length
    MBP.L2=SL(3)*1e-3;%Robot seg2 length
    MBP.Lr=SL(2)*1e-3;%Robot rigid seg length
    MBP.Lg=SL(4)*1e-3;%Robot gipper length
    MBP.r11=[1 0 0]'*MBP.rho1;MBP.r12=[0 1 0]'*MBP.rho1;
    MBP.r21=[1/sqrt(2) 1/sqrt(2) 0]'*MBP.rho2;MBP.r22=[-1/sqrt(2) 1/sqrt(2) 0]'*MBP.rho2;
    MBP.Q1=[skewMatrix(MBP.r11)*e3 skewMatrix(MBP.r12)*e3 skewMatrix(MBP.r21)*e3 skewMatrix(MBP.r22)*e3];
    MBP.Q2=[skewMatrix(MBP.r21)*e3 skewMatrix(MBP.r22)*e3];
    MBP.I1=pi*MBP.d1^4/64;MBP.A1=pi*MBP.d1^2/4;
    MBP.I2=pi*MBP.d2^4/64;MBP.A2=pi*MBP.d2^2/4;
    MBP.G=MBP.E/2/(1+MBP.mu);MBP.J=2*MBP.I1;
    MBP.Ke1=diag([3e8 3e8 MBP.A1*MBP.E]);MBP.Kb1=diag([MBP.E*MBP.I1 MBP.E*MBP.I1 10]);
    MBP.Ke2=diag([3e8 3e8 MBP.A2*MBP.E]);MBP.Kb2=diag([MBP.E*MBP.I2 MBP.E*MBP.I2 10]);
    MBP.discrete_element = 1e-3;
end

if(nargin == 2)
    FM = zeros(12,1);

    MBP.E=55e9;%Rod Young's modules
    MBP.mu=0.33;%Rod Poisson rate
    MBP.d1=0.95e-3;%Rod diameters
    MBP.d2=0.40e-3;
    MBP.rho1=2.5e-3;%Rod pitch circle radius
    MBP.rho2=2.7e-3;%Rod pitch circle radius
    MBP.L=600e-3;%Robot stem length
    MBP.L1=SL(1)*1e-3;%Robot seg1 length
    MBP.L2=SL(3)*1e-3;%Robot seg2 length
    MBP.Lr=SL(2)*1e-3;%Robot rigid seg length
    MBP.Lg=SL(4)*1e-3;%Robot gipper length
    MBP.r11=[1 0 0]'*MBP.rho1;MBP.r12=[0 1 0]'*MBP.rho1;
    MBP.r21=[1/sqrt(2) 1/sqrt(2) 0]'*MBP.rho2;MBP.r22=[-1/sqrt(2) 1/sqrt(2) 0]'*MBP.rho2;
    MBP.Q1=[skewMatrix(MBP.r11)*e3 skewMatrix(MBP.r12)*e3 skewMatrix(MBP.r21)*e3 skewMatrix(MBP.r22)*e3];
    MBP.Q2=[skewMatrix(MBP.r21)*e3 skewMatrix(MBP.r22)*e3];
    MBP.I1=pi*MBP.d1^4/64;MBP.A1=pi*MBP.d1^2/4;
    MBP.I2=pi*MBP.d2^4/64;MBP.A2=pi*MBP.d2^2/4;
    MBP.G=MBP.E/2/(1+MBP.mu);MBP.J=2*MBP.I1;
    MBP.Ke1=diag([3e8 3e8 MBP.A1*MBP.E]);MBP.Kb1=diag([MBP.E*MBP.I1 MBP.E*MBP.I1 10]);
    MBP.Ke2=4*diag([3e8 3e8 MBP.A2*MBP.E]);MBP.Kb2=4*diag([MBP.E*MBP.I2 MBP.E*MBP.I2 10]);
    MBP.discrete_element = 1e-3;
end

if (nargin == 3)
    MBP.E=55e9;%Rod Young's modules
    MBP.mu=0.33;%Rod Poisson rate
    MBP.d1=0.95e-3;%Rod diameters
    MBP.d2=0.40e-3;
    MBP.rho1=2.5e-3;%Rod pitch circle radius
    MBP.rho2=2.7e-3;%Rod pitch circle radius
    MBP.L=600e-3;%Robot stem length
    MBP.L1=SL(1)*1e-3;%Robot seg1 length
    MBP.L2=SL(3)*1e-3;%Robot seg2 length
    MBP.Lr=SL(2)*1e-3;%Robot rigid seg length
    MBP.Lg=SL(4)*1e-3;%Robot gipper length
    MBP.r11=[1 0 0]'*MBP.rho1;MBP.r12=[0 1 0]'*MBP.rho1;
    MBP.r21=[1/sqrt(2) 1/sqrt(2) 0]'*MBP.rho2;MBP.r22=[-1/sqrt(2) 1/sqrt(2) 0]'*MBP.rho2;
    MBP.Q1=[skewMatrix(MBP.r11)*e3 skewMatrix(MBP.r12)*e3 skewMatrix(MBP.r21)*e3 skewMatrix(MBP.r22)*e3];
    MBP.Q2=[skewMatrix(MBP.r21)*e3 skewMatrix(MBP.r22)*e3];
    MBP.I1=pi*MBP.d1^4/64;MBP.A1=pi*MBP.d1^2/4;
    MBP.I2=pi*MBP.d2^4/64;MBP.A2=pi*MBP.d2^2/4;
    MBP.G=MBP.E/2/(1+MBP.mu);MBP.J=2*MBP.I1;
    MBP.Ke1=diag([3e8 3e8 MBP.A1*MBP.E]);MBP.Kb1=diag([MBP.E*MBP.I1 MBP.E*MBP.I1 10]);
    MBP.Ke2=diag([3e8 3e8 MBP.A2*MBP.E]);MBP.Kb2=diag([MBP.E*MBP.I2 MBP.E*MBP.I2 10]);
    MBP.discrete_element = 1e-3;
end

if(nargin == 4)
    MBP.discrete_element = 1e-3;
end
if(nargin == 5)
    MBP.discrete_element = discrete_element*1e-3;
end
Fe=FM(1:3);Me=FM(4:6);
fe=FM(7:9);le=FM(10:12);%distributed not active in this version
QA=zeros(6,1);Res=zeros(5,1);
Guess=zeros(5,1);Guess(2)=-0.001;
%% ====Execute=== %%
% disp(['Target ' num2str(j) ': ' num2str(Tg(1,4)) ' ' num2str(Tg(2,4)) ' ' num2str(Tg(3,4)) ' ']);

[QA,Guess,Res,t1,t2,y1,y2]=shootingOpt2_SingleIK(Tg,Guess,Fe,Me,fe,le,MP);

psi_inv = QA;

%---plot curvatures---%
u=calcCurvature(y1,y2,t1,t2,MBP);
t0 = round(u(1,:)*1000) + u(2,:)/1000;
p0 = [y1(:,1:3);y2(:,1:3)];
S = [p0 t0'];
end

%% ===Model Functions=== %%
function [t,y]=odeCosserat2(y0,v,SegIdx,MP,fe,le,step)
%-----Integral of IVP using difference equation-----
% for BottumUPCosserat_TwoSeg_inv script, inverse differential equations.
% y0- initial (tip) condition
% v- rod elongation strain
% SegIdx- the segment index (1 or 2) you are integrating
% MP- Mechanical properties
% fe, le- external distributed loads
% step- integrating step size
%----------info-----------%
% ver i1.0
% by Yuyang Chen
% date 20200604
%-------------------------------------------------------------------------%
DoF=length(y0);
if(SegIdx == 1)
    Q=MP.Q1;
    N=round(MP.L1/step)+1;
    t=linspace(MP.L1+MP.ell,MP.ell,N)'; % from tip down
    Kb = 4*MP.Kb1+4*MP.Kb2;
    mod_SegIdx=1;
elseif(SegIdx == 2)
    Q=MP.Q2;
    N=round(MP.L2/step)+1;
    t=linspace(MP.L1+MP.Lr+MP.L2+MP.ell,MP.L1+MP.Lr+MP.ell,N)';
    Kb = 4*MP.Kb2;
    mod_SegIdx=0;
elseif(SegIdx == 0)%for base stem
    Q=MP.Q1;
    N=floor(MP.ell/step)+2;
    t=linspace(floor(MP.ell/1e-3),0,N-1)';
    t=t*1e-3;
    t=[MP.ell;t];
    gamma=4.2;
    Kb = 4*MP.Kb1+4*MP.Kb2 + (gamma-1)*8*MP.Kb2;
    mod_SegIdx=1;
end

y=zeros(N,DoF);
y(1,:)=y0';
for i=1:N-1
    p=y(i,1:3)';R=[y(i,4:6);y(i,7:9);y(i,10:12)]';n=y(i,13:15)';m=y(i,16:18)';q=y(i,19:end)';
    u=inv(Kb)*R'*m;u(3)=0;
    
    %constant-curvature-based evolution
    theta=(t(i)-t(i+1))*norm(u);delta=-atan2(u(2),u(1))+pi/2;
    if(theta~=0)
        p_dot=R*( (t(i)-t(i+1))/theta*[cos(delta)*(1-cos(theta)) sin(delta)*(cos(theta)-1) -sin(theta)]' );
        R_dot=[cos(delta)^2*cos(theta)+sin(delta)^2 -sin(delta)*cos(delta)*(cos(theta)-1) cos(delta)*sin(theta);...
               sin(delta)*cos(delta)*(1-cos(theta)) cos(delta)^2+cos(theta)*sin(delta)^2 -sin(delta)*sin(theta);...
              -cos(delta)*sin(theta) sin(delta)*sin(theta) cos(theta)];
    else
        p_dot=R*[0 0 -(t(i)-t(i+1))]';
        R_dot=eye(3);
    end
    p=p+(p_dot);
    R=R*R_dot';

    n_dot=fe*(t(i)-t(i+1));
    m_dot=-S(p_dot)*n + le*(t(i)-t(i+1)) +...
    2*R*(( cross(S(u)*MP.r11,MP.Ke1*v(:,1))+cross(MP.r11,S(u)*MP.Ke1*v(:,1)) + ...
           cross(S(u)*MP.r12,MP.Ke1*v(:,2))+cross(MP.r12,S(u)*MP.Ke1*v(:,2)) )*mod_SegIdx + ...
         ( cross(S(u)*MP.r21,MP.Ke2*v(:,3))+cross(MP.r21,S(u)*MP.Ke2*v(:,3)) + ...
           cross(S(u)*MP.r22,MP.Ke2*v(:,4))+cross(MP.r22,S(u)*MP.Ke2*v(:,4)) ))*(t(i)-t(i+1));
    n=n+n_dot;
    m=m+m_dot;
    
    q_dot=Q'*u*(t(i)-t(i+1));
    q=q+q_dot;
    y(i+1,:)=[p;R(:,1);R(:,2);R(:,3);n;m;q]';
end

end

%---5DoF reverted integrating, considering stem bending
function [Rsd,t1,y1,t2,y2,QA]=invShooting3(Guess,Tg,Fe,Me,fe,le,MP)

Rg=Tg(1:3,1:3);pg=Tg(1:3,4);
v=zeros(3,4);v(end,:)=Guess(1:4)';
MP.ell=Guess(5);
fe=[0 0 0]';nc=Fe;%assume fe=0.
mc=Me-2*( cross(Rg*MP.r21,Rg*MP.Ke2*v(:,3)) + cross(Rg*MP.r22,Rg*MP.Ke2*v(:,4)) );
step=1e-3;

yL2=[pg' Rg(:,1)' Rg(:,2)' Rg(:,3)' nc' mc' 0 0]';
yL2_=[pg'-MP.Lg*[0 0 1]*Rg' Rg(:,1)' Rg(:,2)' Rg(:,3)' nc' mc' 0 0]';
[t2,y2]=odeCosserat2(yL2_,v,2,MP,fe,le,step);
y2=[yL2';y2];
t2=[MP.Lg+MP.L2+MP.Lr+MP.L1+MP.ell;t2];
R1=[y2(end,4:6);y2(end,7:9);y2(end,10:12)]';

yL1=[y2(end,1:18)';0;0;y2(end,19:20)'];
yL1_=[yL1(1:3)'-MP.Lr*[0 0 1]*R1' yL1(4:15)' (yL1(16:18)-2*(cross(R1*MP.r11,R1*MP.Ke1*v(:,1))+cross(R1*MP.r12,R1*MP.Ke1*v(:,2))) )' yL1(19:22)'];

[t1,y1]=odeCosserat2(yL1_,v,1,MP,fe,le,step);
y1=[yL1';y1];
t1=[MP.Lr+MP.L1+MP.ell;t1];
yL0=y1(end,:)';
%R0=[y1(end,4:6);y1(end,7:9);y1(end,10:12)]';
%p0=y1(end,1:3)';

[t0,y0]=odeCosserat2(yL0,v,0,MP,fe,le,step);
y0=[yL0';y0];
t0=[MP.ell;t0];
yL_stem=y0(end,:)';
R0=[y0(end,4:6);y0(end,7:9);y0(end,10:12)]';
p0=y0(end,1:3)';

y1=[y1;y0];t1=[t1;t0];
qe1=(MP.L+MP.L1)*[v(3,1) v(3,2)]';
qe2=(MP.L+MP.L1+MP.L2+MP.Lr)*[v(3,3) v(3,4)]';%not used currently
QA=[[0 0 1]*Logm(R0);MP.ell;y0(end,19:20)'-qe1;y0(end,21:22)'-qe2];

Rsd=zeros(5,1);

Rsd(1:3)=p0;
Rsd(4:5)=[1 0 0;0 1 0]*Logm(R0);
t1=fliplr(t1')';t2=fliplr(t2')';
y1=fliplr(y1')';y2=fliplr(y2')';

end


%% ===inverse kinematics based control
function [QA,Guess,Rsd,t1,t2,y1,y2]=shootingOpt2_SingleIK(Tg,Guess,Fe,Me,fe,le,MP)
    lambda=5e-2;
    eps=1e-4;
    
    N_ukn=length(Guess);
    dGuess=eye(N_ukn)*1e-6;
    count = 0;
    
    [Rsd,t1,y1,t2,y2,QA]=invShooting3(Guess,Tg,Fe,Me,fe,le,MP);
    while(norm(Rsd)>eps)
    
        for i=1:N_ukn%Jacobian(DoF_rsd,DoF_guess)
            [Rsd_]=invShooting3(Guess+dGuess(:,i),Tg,Fe,Me,fe,le,MP);
            J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
        end
    
        Guess=Guess-J'/(J'*J+lambda*eye(N_ukn))*(Rsd);
        if(Guess<0)
            Guess = 0;
        end
        [Rsd,t1,y1,t2,y2,QA]=invShooting3(Guess,Tg,Fe,Me,fe,le,MP);
        
        count=count+1;
    
        if(count>20)
            break;
        end
    end

end

%% ===Other Function=== %%
function [u]=Logm(R)
%simplified calculation for logarithmic map (u in R3)
theta = acos((R(1,1)+R(2,2)+R(3,3)-1)/2);
t=real(theta);
if t==0
    u=[0 0 0]';
else
    u=[R(3,2)-R(2,3) R(1,3)-R(3,1) R(2,1)-R(1,2)]'/2/sin(t)*t;
end

end