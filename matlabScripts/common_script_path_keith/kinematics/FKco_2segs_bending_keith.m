function [Tend, S] = FKco_2segs_bending_keith(qa, SL, FM, MBP, discrete_element)
% Declaration
% the end-effector coordinate {g}.
% the base coordinate {b}
%
% This is a function to calculate the pose of {g} in {b} for a 2-seg continuum
% manipulator Based on the Cosserat rod theory.
%
% input1: qa (6 X 1 vector, for 6 DoFs)
% input2: structural length SL (4 X 1 vector, L1 Lr L2 Lg)
%                             or (8 X 1 vector additional gamma1 gamma2 gamma3 zeta)
% input3: FM (external concentrated/distributed force/moment, 12 X 1 vector)
% input4: multi-backbone manipulator parameter MBP
% input5: discrete_element (the size of the smallest element of integration, 
% also used in plotting)
%
% output1: Tend (4 X 4 matrix)
% output2: S (discrete points on the central curve, N X 4 matrix, last column
%             denotes the curvature on relative point)
%
% Author: Keith W.
% Ver. 1.0
% Date: 14.02.2022

%clc;clear;
%% ===<<Bottom-to-Up Cosserat model, Two Segments>>=== %%
%%====================List of funs and vars====================%%
% (i)Functions:
% shootingOpt-shooting method nested in an optimization framework
% forShooting-guess-residue shooting shell for forward kinematics
% odeCosserat1-(external) universal cosserat integration kernal
% calcCurvature- calculate the curvature at every point given the curve
% calcActuation- calculate the actuation lengths using constant curvature
%model for unloaded shape.
% (ii)Variables:
% MBP-mechanical parameter struct of the robot
% qa-actuation lengths (phi and d can be included)
% Fe, Me-external tip load
% fe, le-distributed load
%%-------------------------------------------------------------%%

%% ===Input Specifications=== %%
e3=[0 0 1]';
qa(2:end) = qa(2:end)/1000;

if(nargin == 1)
    SL = [100 10 20 15]';
    FM = zeros(12,1);
    
    MBP.E=40e9;%Rod Young's modules
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
    MBP.Q1=[skewMatrix_keith(MBP.r11)*e3 skewMatrix_keith(MBP.r12)*e3 skewMatrix_keith(MBP.r21)*e3 skewMatrix_keith(MBP.r22)*e3];
    MBP.Q2=[skewMatrix_keith(MBP.r21)*e3 skewMatrix_keith(MBP.r22)*e3];
    MBP.I1=pi*MBP.d1^4/64;MBP.A1=pi*MBP.d1^2/4;
    MBP.I2=pi*MBP.d2^4/64;MBP.A2=pi*MBP.d2^2/4;
    MBP.G=MBP.E/2/(1+MBP.mu);MBP.J=2*MBP.I1;
    MBP.Ke1=diag([3e8 3e8 MBP.A1*MBP.E]);
    MBP.Kb1=diag([MBP.E*MBP.I1 MBP.E*MBP.I1 2*MBP.G*MBP.I1]);
    MBP.Ke2=4*diag([3e8 3e8 MBP.A2*MBP.E]);
    MBP.Kb2=4*diag([MBP.E*MBP.I2 MBP.E*MBP.I2 2*MBP.G*MBP.I2]);
    MBP.zeta = 0.2;
    MBP.Ls = 0;
    MBP.L1o=MBP.L1;%Robot seg1 length out of trocar
    MBP.Lo=MBP.L;%Robot stem length in trocar
    MBP.discrete_element = 1e-3;
end

if(nargin == 2)
    FM = zeros(12,1);

    MBP.E=40e9;%Rod Young's modules
    MBP.mu=0.33;%Rod Poisson rate
    MBP.d1=0.95e-3;%Rod diameters
    MBP.d2=0.40e-3;
    MBP.rho1=2.5e-3;%Rod pitch circle radius
    MBP.rho2=2.75e-3;%Rod pitch circle radius
    MBP.L=600e-3;%Robot stem length
    MBP.L1=SL(1)*1e-3;%Robot seg1 length
    MBP.L2=SL(3)*1e-3;%Robot seg2 length
    MBP.Lr=SL(2)*1e-3;%Robot rigid seg length
    MBP.Lg=SL(4)*1e-3;%Robot gipper length
    MBP.r11=[1 0 0]'*MBP.rho1;MBP.r12=[0 1 0]'*MBP.rho1;
    MBP.r21=[1/sqrt(2) -1/sqrt(2) 0]'*MBP.rho2;MBP.r22=[1/sqrt(2) 1/sqrt(2) 0]'*MBP.rho2;
    MBP.Q1=[skewMatrix_keith(MBP.r11)*e3 skewMatrix_keith(MBP.r12)*e3 skewMatrix_keith(MBP.r21)*e3 skewMatrix_keith(MBP.r22)*e3];
    MBP.Q2=[skewMatrix_keith(MBP.r21)*e3 skewMatrix_keith(MBP.r22)*e3];
    MBP.I1=pi*MBP.d1^4/64;MBP.A1=pi*MBP.d1^2/4;
    MBP.I2=pi*MBP.d2^4/64;MBP.A2=pi*MBP.d2^2/4;
    MBP.G=MBP.E/2/(1+MBP.mu);MBP.J=2*MBP.I1;
    MBP.Ke1=diag([3e8 3e8 MBP.A1*MBP.E]);MBP.Kb1=diag([MBP.E*MBP.I1 MBP.E*MBP.I1 2*MBP.G*MBP.I1]);
    MBP.Ke2=4*diag([3e8 3e8 MBP.A2*MBP.E]);MBP.Kb2=4*diag([MBP.E*MBP.I2 MBP.E*MBP.I2 2*MBP.G*MBP.I2]);
    MBP.zeta = 0.2;
    MBP.Ls = 0;
    MBP.L1o=MBP.L1;%Robot seg1 length out of trocar
    MBP.Lo=MBP.L;%Robot stem length in trocar
    MBP.discrete_element = 1e-3;
end

if (nargin == 3)
    MBP.E=40e9;%Rod Young's modules
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
    MBP.Q1=[skewMatrix_keith(MBP.r11)*e3 skewMatrix_keith(MBP.r12)*e3 skewMatrix_keith(MBP.r21)*e3 skewMatrix_keith(MBP.r22)*e3];
    MBP.Q2=[skewMatrix_keith(MBP.r21)*e3 skewMatrix_keith(MBP.r22)*e3];
    MBP.I1=pi*MBP.d1^4/64;MBP.A1=pi*MBP.d1^2/4;
    MBP.I2=pi*MBP.d2^4/64;MBP.A2=pi*MBP.d2^2/4;
    MBP.G=MBP.E/2/(1+MBP.mu);MBP.J=2*MBP.I1;
    MBP.Ke1=diag([3e8 3e8 MBP.A1*MBP.E]);MBP.Kb1=diag([MBP.E*MBP.I1 MBP.E*MBP.I1 2*MBP.G*MBP.I1]);
    MBP.Ke2=4*diag([3e8 3e8 MBP.A2*MBP.E]);MBP.Kb2=4*diag([MBP.E*MBP.I2 MBP.E*MBP.I2 2*MBP.G*MBP.I2]);
    MBP.zeta = 0.2;
    MBP.Ls = 0;
    MBP.L1o=MBP.L1;%Robot seg1 length out of trocar
    MBP.Lo=MBP.L;%Robot stem length in trocar
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
if(qa(2)>MBP.L1)
    MBP.Ls = qa(2) - MBP.L1;
else
    MBP.Ls = 0;
    MBP.Lo = MBP.L + MBP.L1 - qa(2);
    MBP.L1o = qa(2);
end

%% ====Execute=== %%
[~,t1,t2,y1,y2]=shootingOpt(qa,Fe,Me,fe,le,MBP);

p=y2(end,1:3)';R=[y2(end,4:6);y2(end,7:9);y2(end,10:12)]';
Tend=[R p*1000;0 0 0 1];

%---plot curvatures---%
u=calcCurvature(y1,y2,t1,t2,MBP);
t0 = round(u(1,:)*1000) + u(2,:)/1000;
p0 = [y1(:,1:3);y2(:,1:3)];
S = [p0*1000 t0'];
end

%% ===Model Functions=== %%
function [Guess,t1,t2,y1,y2]=shootingOpt(qa,Fe,Me,fe,le,MBP)
    dGuess=eye(10)*1e-5;
    lambda=5e-10; % Jacobian damping
    eps=1e-5; % residue tolerance
    
    nc=[0 0 0]';mc=[0 0 0]';v=[0 0 0 0]';Guess=[nc;mc;v]; % guessed base force moment strain
    [Rsd,t1,y1,t2,y2]=forShooting(Guess,qa,Fe,Me,fe,le,MBP);
    
    J= zeros(10,10);
    while(norm(Rsd)>eps)    
        for i=1:10 % finite differencing for Jacobian of initial guess
            [Rsd_]=forShooting(Guess+dGuess(:,i),qa,Fe,Me,fe,le,MBP);
            J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
        end
    
        Guess=Guess-eye(10)/(J'*J+lambda*eye(10))*J'*(Rsd); % update guess
        [Rsd,t1,y1,t2,y2]=forShooting(Guess,qa,Fe,Me,fe,le,MBP); % number IVP of equal to number of guessed values 
    
    end

end

function [Rsd,t1,y1,t2,y2]=forShooting(Guess,qa,Fe,Me,fe,le,MBP)
% do an IVP shooting


nc=Guess(1:3); % base internal force
mc=Guess(4:6); % base internal load
v=zeros(3,4);v(end,:)=Guess(7:10)'; % rod elongation strain

p0=zeros(3,1);R0=eye(3);
y0=[p0' R0(:,1)' R0(:,2)' R0(:,3)' nc' mc' 0 0 0 0]';
if(MBP.Ls)>0
    [t0,y0]=odeCosserat1(y0,v,0,MBP,fe,le,MBP.discrete_element);
    y0_=y0(end,:);
else
    t0 = 0;
    y0_=y0;
end
[t1,y1]=odeCosserat1(y0_,v,1,MBP,fe,le,MBP.discrete_element); % first seg y=p,R,n,m,q for each row
R1=[y1(end,4:6);y1(end,7:9);y1(end,10:12)]'; % R1e
y1(end+1,:)=y1(end,:);y1(end,1:3)=y1(end,1:3)+[0 0 1]*MBP.Lr*R1'; % move from 1e to 2b
t1(end+1)=t1(end)+MBP.Lr;
yL1=y1(end,:)';

y1_=[yL1(1:3);yL1(4:15);yL1(16:18)+2*(cross(R1*MBP.r11,R1*MBP.Ke1*v(:,1))+...
    cross(R1*MBP.r12,R1*MBP.Ke1*v(:,2)));yL1(21:22)]; % ? start of 2nd seg

[t2,y2]=odeCosserat1(y1_,v,2,MBP,fe,le,MBP.discrete_element); % second seg
R2=[y2(end,4:6);y2(end,7:9);y2(end,10:12)]'; % R2e
y2(end+1,:)=y2(end,:);y2(end,1:3)=y2(end,1:3)+[0 0 1]*MBP.Lg*R2'; % move from 2e to g
t2(end+1)=t2(end)+MBP.Lg;
yL2=y2(end,:)';

if(MBP.Ls>0)
    y1 = [y0;y1];
    t1 = [t0;t1];
end
qe1=(MBP.Lo+MBP.L1o)*[v(3,1) v(3,2)]'; % elongation seg1
qe2=(MBP.Lo+MBP.L1o+MBP.L2+MBP.Lr)*[v(3,3) v(3,4)]'; % elongation seg2

Rsd=zeros(10,1);
Rsd(1:3)=Fe-yL2(13:15); % boundary conditions
Rsd(4:6)=Me-yL2(16:18)-2*( cross(R2*MBP.r21,R2*MBP.Ke2*v(:,3))+cross(R2*MBP.r22,R2*MBP.Ke2*v(:,4)));
Rsd(7:8) =yL1(19:20)-(qa(3:4)+qe1); % path length
Rsd(9:10)=yL2(19:20)-(qa(5:6)+qe2);
end

%% ===Other Functions=== %%
function [u]=calcCurvature(y1,y2,t1,t2,MBP)
u_=[0 0 0]';
size1=size(y1(:,1:3));size2=size(y2(:,1:3));
n1=size1(1);n2=size2(1);
u=zeros(3,n1+n2);u_n=zeros(1,n1+n2);
y_all=[y1(:,1:18);y2(:,1:18)];
zeta = MBP.zeta;
n0 = ceil(MBP.Ls/MBP.discrete_element) + 1;
for i =1:n1
    R=[y_all(i,4:6)' y_all(i,7:9)' y_all(i,10:12)'];
%     n=y_all(i,13:15)';
    if(i<n0)
        m=y_all(i,16:18)';
        u(:,i)=eye(3)/((4*MBP.Kb1+4*MBP.Kb2)/zeta)*R'*m+u_;
        u_n(i)=norm(u(:,i));
        u(3,i)=t1(i);
        continue;
    end
    m=y_all(i,16:18)';
    u(:,i)=eye(3)/(4*MBP.Kb1+4*MBP.Kb2)*R'*m+u_;
    u_n(i)=norm(u(:,i));
    u(3,i)=t1(i);
end
for i =n1+1:n1+n2
    R=[y_all(i,4:6)' y_all(i,7:9)' y_all(i,10:12)'];
%     n=y_all(i,13:15)';
    m=y_all(i,16:18)';
    u(:,i)=eye(3)/(4*MBP.Kb2)*R'*m+u_;
    u_n(i)=norm(u(:,i));
    u(3,i)=t2(i-n1);
end
% figure(3);hold on;
% plot(u_n,'-r')

end

% function [R]=Expm(u)
% %simplified calculation for exponetial map (u in R3)
% theta=norm(u);
% if(theta == 0)
%     R=eye(3);
% else
%     un=u/theta;
%     R=cos(theta)*eye(3)+(1-cos(theta))*(un*un')+sin(theta)*skewMatrix_keith(un);
% end
% end

function [t,y,U]=odeCosserat1(y0,v,SegIdx,MBP,fe,le,step)
%-----Integral of IVP using difference equation-------
% for BottumUPCosserat_TwoSeg script, forward differential equations.
% y0- initial condition
% v- rod elongation strain
% SegIdx- the segment index (1 or 2) you are integrating
% MBP- Mechanical properties
% fe, le- external distributed loads
% step- integrating step size
%----------info-----------%
% ver f1.0
% by Yuyang Chen
% date 20200524
%-------------------------------------------------------------------------%
DoF=length(y0);
Kb1=MBP.Kb1;
Ke1=MBP.Ke1;
Kb2=MBP.Kb2;
Ke2=MBP.Ke2;
if(SegIdx == 1)
    Q=MBP.Q1;
    N=ceil(MBP.L1o/step);
    t=linspace(0,MBP.L1o,N)';
    mod_SegIdx=1;
    Kb = 4*Kb1+4*Kb2;
    step=MBP.L1o/(N-1);
elseif(SegIdx == 2)
    Q=MBP.Q2;
    N=ceil(MBP.L2/step);
    t=linspace(MBP.L1o+MBP.Lr,MBP.L1o+MBP.Lr+MBP.L2,N)';
    mod_SegIdx=0;
    Kb = 4*Kb2;
    step=MBP.L2/(N-1);
elseif(SegIdx == 0)
    Q=MBP.Q1;
    N=ceil(MBP.Ls/step);
    t=linspace(0,MBP.Ls,N)';
    mod_SegIdx=1;
    zeta=MBP.zeta;
    Kb = (4*Kb1+4*Kb2)/zeta;
    step=MBP.Ls/(N-1);
end
    
y=zeros(N,DoF);U=zeros(N,3);
y(1,:)=y0';R=[y0(4:6) y0(7:9) y0(10:12)];m=y0(16:18);
for i=1:N-1
    p=y(i,1:3)';R=[y(i,4:6);y(i,7:9);y(i,10:12)]';n=y(i,13:15)';m=y(i,16:18)';q=y(i,19:end)';
%     u=(4*Kb2+mod(SegIdx,2)*4*Kb1)\R'*m;u(3)=0; % torsion free !!!
    u=Kb\R'*m;u(3)=0;
    
    %constant-curvature-based evolution
    theta=step*norm(u);delta=-atan2(u(2),u(1))+pi/2;
    costheta=cos(theta);sintheta=sin(theta);cosdelta=cos(delta);sindelta=sin(delta); % cache
    if(theta~=0)
        p_dot=R*( step/theta*[cosdelta*(1-costheta) sindelta*(costheta-1) sintheta]' );
        R_dot=[cosdelta^2*costheta+sindelta^2 -sindelta*cosdelta*(costheta-1) cosdelta*sintheta;...
               sindelta*cosdelta*(1-costheta) cosdelta^2+costheta*sindelta^2 -sindelta*sintheta;...
              -cosdelta*sintheta sindelta*sintheta costheta];
    else
        p_dot=R*[0 0 step]';
        R_dot=eye(3);
    end
    p=p+p_dot;
    R=R*R_dot;

    n_dot=-fe*step;
    m_dot=-skewMatrix_keith(p_dot)*n - le*step -...
    2*R*(( cross(skewMatrix_keith(u)*MBP.r11,Ke1*v(:,1))+cross(MBP.r11,skewMatrix_keith(u)*Ke1*v(:,1)) + ...
           cross(skewMatrix_keith(u)*MBP.r12,Ke1*v(:,2))+cross(MBP.r12,skewMatrix_keith(u)*Ke1*v(:,2)) )*mod_SegIdx + ...
         ( cross(skewMatrix_keith(u)*MBP.r21,Ke2*v(:,3))+cross(MBP.r21,skewMatrix_keith(u)*Ke2*v(:,3)) + ...
           cross(skewMatrix_keith(u)*MBP.r22,Ke2*v(:,4))+cross(MBP.r22,skewMatrix_keith(u)*Ke2*v(:,4)) ))*step;
    n=n+n_dot;
    m=m+m_dot;
    
    q_dot=Q'*u*step;
    q=q+q_dot;
    y(i+1,:)=[p;R(:,1);R(:,2);R(:,3);n;m;q]';
    U(i,:)=u';
end
U(end,:)=Kb\R'*m;
end


