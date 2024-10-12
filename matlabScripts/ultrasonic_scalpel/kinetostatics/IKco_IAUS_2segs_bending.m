function [qa,S, Tend] = IKco_IAUS_2segs_bending(T_tip, MBP)
% Declaration
% the end-effector coordinate of continuum instrument {g}.
% the base coordinate of continuum instrument {b}
% the tip of In-vivo Assembled Ultrasonic Scalpel (IAUS) {t} which is rigid
% attached to {g}
% 
% 
% This is a function to calculate the actuation of a 2-seg continuum
% manipulator given {t} Based on the Cosserat rod theory.
%
% input1: T_tip (4 X 4 matrix, the pos of {t})
% input2: multi-backbone manipulator parameter MBP
% input3: discrete_element (the size of the smallest element of integration, 
% also used in plotting)
%
% output1: Qa (6 X 1 vector, for 6 DoFs) (rad, mm)
% output2: S (discrete points on the central curve, N X 4 matrix, last column
%             denotes the curvature on relative point)
%
% Author: Keith W.
% Ver. 1.0
% Date: 14.02.2022

%% Bottom-to-Up Cosserat model, Two Segments
% (i)Functions:
% shootingOpt-shooting method nested in an optimization framework
% forShooting-guess-residue shooting shell for forward kinematics
% odeCosserat1-(external) universal cosserat integration kernal
% calcCurvature- calculate the curvature at every point given the curve
% model for unloaded shape.
% (ii)Variables:
% MBP-mechanical parameter struct of the robot
% qa-actuation lengths (phi and d can be included)
% Fe, Me-external tip load
% fe, le-distributed load
%%-------------------------------------------------------------%%



%% Input Specifications
% R_tip = T_tip(1:3,1:3);
% P_tip = T_tip(1:3,4);

if (nargin == 2)
    MBP.discrete_element = 1e-3;
end

if(nargin == 3)
    MBP.discrete_element = 1e-3;
end

Fe=zeros(3,1);Me=zeros(3,1);
fe=zeros(3,1);le=zeros(3,1);%distributed not active in this version

% 猜测的驱动
qa = [0;200e-3;0;0;0;0];
if(qa(2)>MBP.L1)
    MBP.Ls = qa(2) - MBP.L1;
else
    MBP.Ls = 0;
    MBP.L1o = qa(2);
end

%% Execution
[guess,t1,t2,y1,y2]=shootingOpt(qa,T_tip,fe,le,MBP);

p=y2(end,1:3)';R=[y2(end,4:6);y2(end,7:9);y2(end,10:12)]';
Tend=[R p*1000;0 0 0 1];

qa = [guess(11:12);0;0;0;0];


%---plot curvatures---%
u=calcCurvature(y1,y2,t1,t2,MBP);
t0 = round(u(1,:)*1000) + u(2,:)/1000;
p0 = [y1(:,1:3);y2(:,1:3)];
S = [p0*1000 t0'];
end

%% Model Functions
function [Guess,t1,t2,y1,y2]=shootingOpt(qa,T_tip,fe,le,MBP)
    dGuess=eye(12)*1e-5;
    lambda=5e-10; % Jacobian damping
    eps=1e-7; % residue tolerance
    eps_ = 1e-4; % 能量阈值
    
    nc=[0 0 0]';mc=[0 0 0]';v=[0 0 0 0]';Guess=[nc;mc;v;qa(1:2)]; % guessed base force moment strain
    [Rsd,t1,y1,t2,y2,F_port] = forShooting(Guess,T_tip,fe,le,MBP);
    F_port_old = F_port;
    
    J = zeros(10,12);
    J_ = zeros(1,12);
    while(norm(Rsd)>eps) 
        for i=1:12 % finite differencing for Jacobian of initial guess
            [Rsd_,~,~,~,~,F_port_]=forShooting(Guess+dGuess(:,i),T_tip,fe,le,MBP);
            J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
            J_(:,i) = (F_port_ - F_port)/norm(dGuess(:,i));
        end
        
        if (norm(Rsd) > eps_)
            Guess=Guess-eye(12)/(J'*J+lambda*eye(12))*J'*(Rsd); % update guess
            [Rsd,t1,y1,t2,y2,F_port] = forShooting(Guess,T_tip,fe,le,MBP); % number IVP of equal to number of guessed values 
            disp(['norm(Rsd) =' num2str(norm(Rsd))]); 
            disp(['norm(F_port) =' num2str(F_port)]); 
        else
            Guess=Guess-eye(12)/(J'*J+lambda*eye(12))*J'*(Rsd) - ...
                (eye(12)-eye(12)/(J'*J+lambda*eye(12))*J'*J)*0.01*J_'; % update guess
            [Rsd,t1,y1,t2,y2,F_port] = forShooting(Guess,T_tip,fe,le,MBP); % number IVP of equal to number of guessed values 
            disp(['norm(Rsd) =' num2str(norm(Rsd))]); 
            disp(['norm(F_port) =' num2str(F_port)]); 
            if (F_port >= F_port_old)
                break
            else
                F_port_old = F_port;
            end
        end
    end
end

function [Rsd,t1,y1,t2,y2,F_port]=forShooting(Guess,T_tip,fe,le,MBP)
% do an IVP shooting


nc=Guess(1:3); % base internal force
mc=Guess(4:6); % base internal load
v=zeros(3,4);v(end,:)=Guess(7:10)'; % rod elongation strain

p0=zeros(3,1);R0=RotmAxisZ(Guess(11));
y0=[p0' R0(:,1)' R0(:,2)' R0(:,3)' nc' mc' 0 0 0 0]';

if(Guess(12)>MBP.L1)
    MBP.Ls = Guess(12) - MBP.L1;
else
    MBP.Ls = 0;
    MBP.L1o = Guess(12);
end

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

y1_=[yL1(1:3);yL1(4:15);cross(R1*[0 0 MBP.Lr]',yL1(13:15)) + yL1(16:18)+2*(cross(R1*MBP.r11,R1*MBP.Ke1*v(:,1))+...
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
qe1=(MBP.Lstem+MBP.L1)*[v(3,1) v(3,2)]'; % elongation seg1
qe2=(MBP.Lstem+MBP.L1+MBP.L2+MBP.Lr)*[v(3,3) v(3,4)]'; % elongation seg2

Rsd=zeros(10,1);

% 位姿约束
b_P_g = yL2(1:3);
b_R_g = R2;
b_P_tip = T_tip(1:3,4)*1e-3;
b_R_tip = T_tip(1:3,1:3);

Rsd(1:3) = b_P_g + b_R_g * MBP.g_P_tip - b_P_tip;
[~,err_Rot] = error_orientation(b_R_g * MBP.g_R_tip, b_R_tip) ;
Rsd(4:6) = err_Rot;

tip_P = b_R_tip\(b_P_tip - MBP.b_P_port);
L_in = tip_P(3);

%力学约束
b_F_end = yL2(13:15);
b_M_end = yL2(16:18) + 2*( cross(R2*MBP.r21,R2*MBP.Ke2*v(:,3))+ ...
    cross(R2*MBP.r22,R2*MBP.Ke2*v(:,4))) - cross(R2*[0 0 -MBP.Lg]',yL2(13:15));

tip_F_port = b_R_tip\(-MBP.b_G - b_F_end);
Rsd(7) = tip_F_port(3);
Rsd(8:10) = b_M_end + cross(b_R_tip*[0;0;L_in] + b_P_g - b_P_tip,b_F_end) + cross(b_R_tip*[0;0;L_in - MBP.L_gravity],MBP.b_G);

%返回切口侧向力
F_port = norm(tip_F_port);

end

%% Other Functions
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
    Kb = 4*Kb1+16*Kb2+MBP.K1*Kb1;
    step=MBP.L1o/(N-1);
elseif(SegIdx == 2)
    Q=MBP.Q2;
    N=ceil(MBP.L2/step);
    t=linspace(MBP.L1o+MBP.Lr,MBP.L1o+MBP.Lr+MBP.L2,N)';
    mod_SegIdx=0;
    Kb = 16*Kb2+MBP.K2*Kb1;
    step=MBP.L2/(N-1);
elseif(SegIdx == 0)
    Q=MBP.Q1;
    N=ceil(MBP.Ls/step);
    t=linspace(0,MBP.Ls,N)';
    mod_SegIdx=1;
    zeta=MBP.zeta;
    Kb = (4*Kb1+16*Kb2+MBP.K1*Kb1)/zeta;
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


