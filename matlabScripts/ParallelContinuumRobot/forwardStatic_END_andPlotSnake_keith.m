function [Tend,u,Guess]=forwardStatic_END_andPlotSnake_keith(psi, arm_serial, ET,is_plot)
% Declaration
% the end-effector coordinate {g}.
% the base coordinate {b}
%
% This is a function to calculate the pose of {g} in {b}, firstly.
% Secondly, this is a static model.
% Author: Keith W.
% Ver. 1.0
% Date: 07.29.2022

if(nargin == 0)
    psi=[0 0 0 0 0 0 0]';
    is_plot=0;
    arm_serial=getArmPara;
end
if(nargin == 1)
    is_plot=0;
    arm_serial=getArmPara;
end
if(nargin == 2)
    is_plot=0;
end
%% 基础知识
psi([4 6]) = -psi([4 6]) ; 
MP = getMP(arm_serial,psi(1));

u = fromPsi2Curvature(psi,arm_serial.size_para);
qa = fC2M(u,MP);
qa = [0 0 qa]';
ET = reshape(ET, [12 1]);
Fe=ET(1:3);Me=ET(4:6);
fe=ET(7:9);le=ET(10:12);%distributed not active in this version


%% 力学模型
[Guess,t1,t2,y1,y2]=shootingOpt(qa,Fe,Me,fe,le,MP);
u=calcCurvature(y1,y2,t1,t2,MP);
if(is_plot)
    size1=size(y1(:,1:3));size2=size(y2(:,1:3));
    n1=size1(1);n2=size2(1);
    Te = plotSnake([y1(:,1:3);y2(:,1:3)],...
        [y1(:,4:12);y2(:,4:12)],[n1 n2], arm_serial,psi,u,1);
else
    size1=size(y1(:,1:3));size2=size(y2(:,1:3));
    n1=size1(1);n2=size2(1);
    Te = plotSnake([y1(:,1:3);y2(:,1:3)],...
        [y1(:,4:12);y2(:,4:12)],[n1 n2], arm_serial,psi,u,0);
end
p=y2(end,1:3)';R=[y2(end,4:6);y2(end,7:9);y2(end,10:12)]';
Tend=[R p;0 0 0 1];



[F_est,t1est,t2est,y1est,y2est]=shootingOptEst(qa(3:6),Tend,Guess,fe,le,MP);

%---plot curvatures---%

u1=u(:,1);u2=u(:,end);

block_size = round(MP.L1*1000);
u11 = zeros(3,block_size);
u12 = zeros(3,block_size);
u21 = zeros(3,block_size);
u22 = zeros(3,block_size);
v11 = zeros(1,block_size);
v12 = zeros(1,block_size);
v21 = zeros(1,block_size);
v22 = zeros(1,block_size);

bt_11 = zeros(1,block_size);
bt_12 = zeros(1,block_size);
bt_21 = zeros(1,block_size);
bt_22 = zeros(1,block_size);

strain_upper11 = zeros(1,block_size);
strain_upper12 = zeros(1,block_size);
strain_upper21 = zeros(1,block_size);
strain_upper22 = zeros(1,block_size);
strain_lower11 = zeros(1,block_size);
strain_lower12 = zeros(1,block_size);
strain_lower21 = zeros(1,block_size);
strain_lower22 = zeros(1,block_size);
for i=1:block_size
    u11(:,i)=u(:,i)/(1-[0 0 1]*cross(u(:,i),MP.r11));v11(1,i)=Guess(7);
    u12(:,i)=u(:,i)/(1-[0 0 1]*cross(u(:,i),MP.r12));v12(1,i)=Guess(8);
    u21(:,i)=u(:,i)/(1-[0 0 1]*cross(u(:,i),MP.r21));v21(1,i)=Guess(9);
    u22(:,i)=u(:,i)/(1-[0 0 1]*cross(u(:,i),MP.r22));v22(1,i)=Guess(10);
    bt_11(1,i) = norm(u11(:,i))*MP.d1/2;
    bt_12(1,i) = norm(u12(:,i))*MP.d1/2;
    bt_21(1,i) = norm(u21(:,i))*MP.d2/2;
    bt_22(1,i) = norm(u22(:,i))*MP.d2/2;
    strain_upper11(i) = v11(i)+bt_11(i);strain_lower11(i) = v11(i)-bt_11(i);
    strain_upper12(i) = v12(i)+bt_12(i);strain_lower12(i) = v12(i)-bt_12(i);
    strain_upper21(i) = v21(i)+bt_21(i);strain_lower21(i) = v21(i)-bt_21(i);
    strain_upper22(i) = v22(i)+bt_22(i);strain_lower22(i) = v22(i)-bt_22(i);
end

end
%% 辅助函数
% 从psi计算曲率
function u=fromPsi2Curvature(psi,x)
% Ver. 0.2
% added zeta
    l1 = psi(1);
    if(l1<1e-3)
        l1 = 0.01;
    end
    psi = reshape(psi,[1 max(size(psi))]);
    u = zeros(size(psi,1),6);
    uc=zeros(1,6);
    if(x(1)<l1)
        L1=x(1)*1e-3;
    else
        L1= l1*1e-3;
    end
    L2=x(3)*1e-3;
    for i = 1:size(psi,1)
        if size(psi,2)==4
            temPsi=[0 0 psi(i,:)];
        else
            temPsi=psi(i,:);
        end
        
        uc(1)=temPsi(3)/L1*cos(pi/2-temPsi(4));
        uc(2)=temPsi(3)/L1*sin(pi/2-temPsi(4));
        uc(4)=temPsi(5)/L2*cos(pi/2-temPsi(6));
        uc(5)=temPsi(5)/L2*sin(pi/2-temPsi(6));
        u(i,:)=uc;
    end
end

% 从曲率到驱动量
function qa = fC2M(u,MP)
    dGamma = zeros(4,6);
    Ell=zeros(4,4);
    Theta=zeros(6,4);
    K=zeros(6,6);
    
    
    dGamma(1:4,1:3)=(MP.L1+MP.L0*MP.gamma)*MP.Q1';
    dGamma(3:4,4:6)=MP.L2*MP.Q2';
    
    Ell(1:2,1:2)=diag([1 1])*(MP.L1+MP.L);
    Ell(3:4,3:4)=diag([1 1])*(MP.L2+MP.Lr+MP.L1+MP.L+MP.Lcnla +MP.Lprox);
    
    Theta(1:3,1:4)=-MP.A1*MP.E*MP.Q1;
    Theta(1:3,3:4)=-MP.A1*MP.E*MP.Q2*0;
    Theta(4:6,3:4)=-MP.A2*MP.E*MP.Q2;
    


    K1 = MP.K1;
    K2 = MP.K2;


    K(1:3,1:3)=4*MP.Kb1+16*MP.Kb2 + K1 * MP.Kb1;
    K(1:3,4:6)=-16*MP.Kb2 - K2 * MP.Kb1;
    K(4:6,4:6)=16*MP.Kb2 + K2 * MP.Kb1;
    
    G = dGamma-Ell*pinv(Theta)/2*K; % transfer matrix  ==> from u -> q
    qa = G*u';
    qa = qa';
end

% 计算力学参数
function MP = getMP(arm_serial, l1)
    
    size_para = arm_serial.size_para;
    assembly_para = arm_serial.assembly_para;
    stiffness_para = arm_serial.stiffness_para;
    if(size_para(1)>l1)
        L1=l1;
        flag = 0;
    else
        L1=size_para(1);
        flag = 1;       %flag是指地日柔性段是否完全伸出，是对应1
    end
    e3 = [0 0 1]';
    MP.E=arm_serial.material_para(1);                          %Rod Young's modules
    MP.mu=arm_serial.material_para(2);                     %Rod Poisson rate
    MP.G=MP.E/2/(1+MP.mu);          %Rod modules of rigidity
    MP.d1=assembly_para(9)*1e-3;                  %Rod 1 diameter
    MP.d2=assembly_para(10)*1e-3;                   %Rod 2 diameter
    MP.rho1=assembly_para(1)*1e-3*0.5;               %Rod 1 pitch circle radius
    MP.rho2=assembly_para(3)*1e-3*0.5;               %Rod 2 pitch circle radius
    if(flag)
        MP.L=size_para(5)*1e-3;                    %Robot stem length
        MP.L1=size_para(1)*1e-3;                 %Robot seg1 length
    else
        MP.L=size_para(5)*1e-3 + (size_para(1)-l1)*1e-3;                    %Robot stem length
        MP.L1=l1*1e-3;                 %Robot seg1 length
    end
    MP.L2=size_para(3)*1e-3;                 %Robot seg2 length
    MP.Lr=size_para(2)*1e-3;                   %Robot rigid seg length
    MP.Lg=size_para(4)*1e-3;                 %Robot gipper length
    
    if(flag)
        MP.L0=l1-L1;
    else
        MP.L0 = 0;
    end
    MP.Lcnla = 200e-3;
    MP.Lprox = 18e-3;
    MP.delta_t = -45*pi/180;  % 60 degree
    if(arm_serial.port == 3)
        MP.gamma =0;
    else
        MP.gamma =stiffness_para(3);
    end
    MP.K1 = stiffness_para(1);
    MP.K2 = stiffness_para(2);
    MP.r21=[cos(MP.delta_t) sin(MP.delta_t) 0]'*MP.rho2;
    MP.r22=[cos(-MP.delta_t) sin(-MP.delta_t) 0]'*MP.rho2;
    
    MP.r11=[1 0 0]'*MP.rho1;
    MP.r12=[0 1 0]'*MP.rho1;  
    
    MP.Q1=[S(MP.r11)*e3 S(MP.r12)*e3 S(MP.r21)*e3 S(MP.r22)*e3];
    MP.Q2=[S(MP.r21)*e3 S(MP.r22)*e3];
    
    MP.I1=pi*MP.d1^4/64;
    MP.I2=pi*MP.d2^4/64;
    
    MP.A1=pi*MP.d1^2/4;
    MP.A2=pi*MP.d2^2/4 *4 ;
    
    MP.J1=2*MP.I1;
    MP.J2=2*MP.I2;
    
    MP.Ke1=diag([MP.G*MP.A1 MP.G*MP.A1 MP.A1*MP.E]);
    MP.Ke2=diag(1*[MP.G*MP.A2 MP.G*MP.A2 MP.A2*MP.E]);
    
    MP.Kb1=diag([MP.E*MP.I1 MP.E*MP.I1 2*MP.G*MP.I1]);
    MP.Kb2=1*diag([MP.E*MP.I2 MP.E*MP.I2 2*MP.G*MP.I2]);

end

% 
function [Guess,t1,t2,y1,y2]=shootingOpt(qa,Fe,Me,fe,le,MP)
dGuess=eye(10)*1e-5;
lambda=5e-10; % Jacobian damping
eps=1e-5; % residue tolerance

nc=[0 0 0]';mc=[0 0 0]';v=[0 0 0 0]';Guess=[nc;mc;v]; % guessed base force moment strain
[Rsd,t1,y1,t2,y2]=forShooting(Guess,qa,Fe,Me,fe,le,MP);

tic
J= zeros(10,10);
while(norm(Rsd)>eps)
    disp(['residue0 = ' num2str(norm(Rsd))]);

    for i=1:10 % finite differencing for Jacobian of initial guess
        [Rsd_]=forShooting(Guess+dGuess(:,i),qa,Fe,Me,fe,le,MP);
        J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
    end

    Guess=Guess-eye(10)/(J'*J+lambda*eye(10))*J'*(Rsd); % update guess
    [Rsd,t1,y1,t2,y2]=forShooting(Guess,qa,Fe,Me,fe,le,MP); % number IVP of equal to number of guessed values
     toc   
end


end

function [Rsd,t1,y1,t2,y2]=forShooting(Guess,qa,Fe,Me,fe,le,MP)
% do an IVP shooting
if(length(qa)==4)
    p0=zeros(3,1);R0=eye(3);
else
    p0=[0 0 qa(2)]';R0=Expm([0 0 qa(1)]');
end

nc=Guess(1:3); % base internal force
mc=Guess(4:6); % base internal load
v=zeros(3,4);v(end,:)=Guess(7:10)'; % rod elongation strain

y0=[p0' R0(:,1)' R0(:,2)' R0(:,3)' nc' mc' 0 0 0 0]';
[t1,y1]=odeCosserat1(y0,v,1,MP,fe,le,1e-3); % first seg y=p,R,n,m,q for each row
R1=[y1(end,4:6);y1(end,7:9);y1(end,10:12)]'; % R1e
y1(end+1,:)=y1(end,:);y1(end,1:3)=y1(end,1:3)+[0 0 1]*MP.Lr*R1'; % move from 1e to 2b
t1(end+1)=t1(end)+MP.Lr;
yL1=y1(end,:)';

y1_=[yL1(1:3);yL1(4:15);yL1(16:18)+2*(cross(R1*MP.r11,R1*MP.Ke1*v(:,1))+cross(R1*MP.r12,R1*MP.Ke1*v(:,2)));yL1(21:22)]; % ? start of 2nd seg

[t2,y2]=odeCosserat1(y1_,v,2,MP,fe,le,1e-3); % second seg
R2=[y2(end,4:6);y2(end,7:9);y2(end,10:12)]'; % R2e
y2(end+1,:)=y2(end,:);y2(end,1:3)=y2(end,1:3)+[0 0 1]*MP.Lg*R2'; % move from 2e to g
t2(end+1)=t2(end)+MP.Lg;
yL2=y2(end,:)';

qe1=(MP.L+MP.L1)*[v(3,1) v(3,2)]'; % elongation seg1
qe2=(MP.Lcnla +MP.Lprox+MP.L+MP.L1+MP.L2+MP.Lr)*[v(3,3) v(3,4)]'; % elongation seg2

Rsd=zeros(10,1);
Rsd(1:3)=Fe-yL2(13:15); % boundary conditions
Rsd(4:6)=Me-yL2(16:18)-2*( cross(R2*MP.r21,R2*MP.Ke2*v(:,3))+cross(R2*MP.r22,R2*MP.Ke2*v(:,4)));
Rsd(7:8) =yL1(19:20)-(qa(3:4)+qe1); % path length
Rsd(9:10)=yL2(19:20)-(qa(5:6)+qe2);
end

function [F_est,t1,t2,y1,y2]=shootingOptEst(qa,Tg,Guess,fe,le,MP)
% Force estimation give qa and Tg
lambda=5e-9;
eps=1e-4;

% nc=[0 0 0]';
% nc=Fe0;
% mc=[0 0 0]';v=[0 0 0 0]';
%Guess=[nc(1:2);mc;v];
% Guess=[nc;mc;v];
N_ukn=length(Guess);dGuess=eye(N_ukn)*1e-5;

[Rsd,t1,y1,t2,y2]=estShooting(Guess,qa,Tg,fe,le,MP);

k=0;
while(norm(Rsd)>eps)
    k=k+1;
%     disp(['residue0 = ' num2str(norm(Rsd))]);
    
    J=zeros(length(Rsd),N_ukn);
    for i=1:N_ukn
        [Rsd_]=estShooting(Guess+dGuess(:,i),qa,Tg,fe,le,MP);
        J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
    end
    Guess=Guess-(J'*J+lambda*eye(N_ukn))\J'*(Rsd);
    [Rsd,t1,y1,t2,y2]=estShooting(Guess,qa,Tg,fe,le,MP);

    if k==10&&norm(Rsd)>eps
        disp(['Est no converge. residue0 = ' num2str(norm(Rsd))]);
        break
    end
end
F_est=Guess(1:3);
end

function [Rsd,t1,y1,t2,y2]=estShooting(Guess,qa,Tg,fe,le,MP)
if(length(qa)==4)
    p0=zeros(3,1);R0=eye(3);
else
    p0=[0 0 qa(2)]';R0=Expm([0 0 qa(1)]');
end
pg=Tg(1:3,4);Rg=Tg(1:3,1:3);

nc=Guess(1:3);
mc=Guess(4:6);
v=zeros(3,4);v(end,:)=Guess(7:10)';
step=1e-3;

y0=[p0' R0(:,1)' R0(:,2)' R0(:,3)' nc' mc' 0 0 0 0]';
[t1,y1]=odeCosserat1(y0,v,1,MP,fe,le,step);
R1=[y1(end,4:6);y1(end,7:9);y1(end,10:12)]';
y1(end+1,:)=y1(end,:);y1(end,1:3)=y1(end,1:3)+[0 0 1]*MP.Lr*R1';
t1(end+1)=t1(end)+MP.Lr;
yL1=y1(end,:)';

y1_=[yL1(1:3);yL1(4:15);yL1(16:18)+2*(cross(R1*MP.r11,R1*MP.Ke1*v(:,1))+cross(R1*MP.r12,R1*MP.Ke1*v(:,2)));yL1(21:22)];

[t2,y2]=odeCosserat1(y1_,v,2,MP,fe,le,step);
R2=[y2(end,4:6);y2(end,7:9);y2(end,10:12)]';
y2(end+1,:)=y2(end,:);y2(end,1:3)=y2(end,1:3)+[0 0 1]*MP.Lg*R2';
t2(end+1)=t2(end)+MP.Lg;
yL2=y2(end,:)';

qe1=(MP.L+MP.L1)*[v(3,1) v(3,2)]';
qe2=(MP.Lcnla +MP.Lprox + MP.L+MP.L1+MP.L2+MP.Lr)*[v(3,3) v(3,4)]';

Rsd=zeros(10,1);
 % boundary conditions
Rsd(1:3)=yL2(1:3,:)-pg; % end pose
Rsd(4:6)=Logm(R2'*Rg);
Rsd(7:8) =yL1(19:20)-(qa(1:2)+qe1); % path length
Rsd(9:10)=yL2(19:20)-(qa(3:4)+qe2);
end

function [t,y,U]=odeCosserat1(y0,v,SegIdx,MP,fe,le,step)
%-----Integral of IVP using difference equation-------
% for BottumUPCosserat_TwoSeg script, forward differential equations.
% y0- initial condition
% v- rod elongation strain
% SegIdx- the segment index (1 or 2) you are integrating
% MP- Mechanical properties
% fe, le- external distributed loads
% step- integrating step size
%----------info-----------%
% ver f1.0
% by Yuyang Chen
% date 20200524
%-------------------------------------------------------------------------%
DoF=length(y0);
Kb1=MP.Kb1;
Ke1=MP.Ke1;
Kb2=MP.Kb2;
Ke2=MP.Ke2;
K1 = MP.K1;
K2 = MP.K2;

if(SegIdx == 1)
    Q=MP.Q1;
    N=round(MP.L1/step)+1;
    t=linspace(0,MP.L1,N)';
    mod_SegIdx=1;
    Kb = 4*Kb1+16*Kb2+K1 * Kb1;
    step=MP.L1/(N-1);
elseif(SegIdx == 2)
    Q=MP.Q2;
    N=round(MP.L2/step)+1;
    t=linspace(MP.L1+MP.Lr,MP.L1+MP.Lr+MP.L2,N)';
    mod_SegIdx=0;
    Kb = 16*Kb2 + K2*Kb1;
    step=MP.L2/(N-1);
elseif(SegIdx == 0)
    Q=MP.Q1;
    N=round(MP.ell/step)+1;
    t=linspace(0,MP.ell,N)';
    mod_SegIdx=1;
%     gamma=4.2;
    gamma=1;
    Kb = 4*Kb1+16*Kb2 + (gamma-1)*8*Kb2;
    step=MP.ell/(N-1);
end
    
y=zeros(N,DoF);U=zeros(N,3);
y(1,:)=y0';R=[y0(4:6) y0(7:9) y0(10:12)];m=y0(16:18);
for i=1:N-1
    p=y(i,1:3)';R=[y(i,4:6);y(i,7:9);y(i,10:12)]';n=y(i,13:15)';m=y(i,16:18)';q=y(i,19:end)';
    u=Kb\R'*m;u(3)=0;
    
    theta=step*norm(u);delta=(-atan2(u(2),u(1))+pi/2);
    costheta=cos(theta);sintheta=sin(theta);cosdelta=cos(delta);sindelta=sin(delta); % cache
    if(theta~=0)
        p_dot=R*( step/theta*[cosdelta*(1-costheta) sindelta*(costheta-1) sintheta]' );
        R_dot=[cosdelta^2*costheta+sindelta^2 -sindelta*cosdelta*(costheta-1) cosdelta*sintheta;...
               sindelta*cosdelta*(1-costheta) cosdelta^2+costheta*sindelta^2 -sindelta*sintheta;...
              -cosdelta*sintheta sindelta*sintheta costheta];
%         R_dot=[cosdelta^2*costheta+sindelta^2 sindelta*cosdelta*(costheta-1) cosdelta*sintheta;...
%                -sindelta*cosdelta*(1-costheta) cosdelta^2+costheta*sindelta^2 sindelta*sintheta;...
%               -cosdelta*sintheta -sindelta*sintheta costheta];
    else
        p_dot=R*[0 0 step]';
        R_dot=eye(3);
    end
    p=p+p_dot;
    R=R*R_dot;

    n_dot=-fe*step;
    m_dot=-S(p_dot)*n - le*step -...
    2*R*(( cross(S(u)*MP.r11,Ke1*v(:,1))+cross(MP.r11,S(u)*Ke1*v(:,1)) + ...
           cross(S(u)*MP.r12,Ke1*v(:,2))+cross(MP.r12,S(u)*Ke1*v(:,2)) )*mod_SegIdx + ...
         ( cross(S(u)*MP.r21,Ke2*v(:,3))+cross(MP.r21,S(u)*Ke2*v(:,3)) + ...
           cross(S(u)*MP.r22,Ke2*v(:,4))+cross(MP.r22,S(u)*Ke2*v(:,4)) ))*step;
    n=n+n_dot;
    m=m+m_dot;
    
    q_dot=Q'*u*step;
    q=q+q_dot;
    y(i+1,:)=[p;R(:,1);R(:,2);R(:,3);n;m;q]';
    U(i,:)=u';
end
U(end,:)=Kb\R'*m;
end

%% 其他函数

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

function [R]=Expm(u)
%simplified calculation for exponetial map (u in R3)
theta=norm(u);
if(theta == 0)
    R=eye(3);
else
    un=u/theta;
    R=cos(theta)*eye(3)+(1-cos(theta))*(un*un')+sin(theta)*S(un);
end
end

function [u]=calcCurvature(y1,y2,t1,t2,MP)
u_=[0 0 0]';
size1=size(y1(:,1:3));size2=size(y2(:,1:3));
n1=size1(1);n2=size2(1);
u=zeros(3,n1+n2);u_n=zeros(1,n1+n2);
y_all=[y1(:,1:18);y2(:,1:18)];
for i =1:n1
    R=[y_all(i,4:6)' y_all(i,7:9)' y_all(i,10:12)'];
    m=y_all(i,16:18)';
    u(:,i)=eye(3)/(4*MP.Kb1+16*MP.Kb2 + MP.K1*MP.Kb1)*R'*m+u_;
    u_n(i)=norm(u(:,i));
    u(3,i)=t1(i);
end
for i =n1+1:n1+n2
    R=[y_all(i,4:6)' y_all(i,7:9)' y_all(i,10:12)'];
    m=y_all(i,16:18)';
    u(:,i)=eye(3)/(16*MP.Kb2+ MP.K2*MP.Kb1)*R'*m+u_;
    u_n(i)=norm(u(:,i));
    u(3,i)=t2(i-n1);
end

end

function T = S( p )
%this function gives the skew-symmetric matrix of the vector p

if(length(p)==3)
    T=[0 -p(3) p(2); p(3) 0 -p(1);-p(2) p(1) 0];
elseif(length(p)==6)
    R=[0 -p(6) p(5); p(6) 0 -p(4);-p(5) p(4) 0];
    T=[R p(1:3);zeros(1,4)];
end
end

%% plot 函数
function Te = plotSnake(p,Rs,N, arm_serial, psi,u, is_plot)
    grid on;axis equal;hold on;
    
    p = p*1e3;

    Q1 = arm_serial.distribution_para.Q1;
    Q2 = arm_serial.distribution_para.Q2;
    N1=round(N(1));N2=round(N(2));
    num_1 = max(size(Q1));
    num_2 = max(size(Q2));
    
    SP = getStructurePara_keith;
    
    phi = psi(2);
    gamma1 = arm_serial.size_para(6);
    % PHI
    r_phi = [cos(phi+gamma1) -sin(phi+gamma1) 0 
    sin(phi+gamma1) cos(phi+gamma1) 0 
    0 0 1 ];
    init_pose = SP.init_pose(:,:,arm_serial.port);
    R_ip = init_pose(1:3,1:3)*r_phi;
    P_ip = init_pose(1:3,4);


    Te=[R_ip*[Rs(N2+N1,1:3);Rs(N2+N1,4:6);Rs(N2+N1,7:9)]' (P_ip+R_ip*p(N2+N1,:)');[0 0 0 1]];
    if(~is_plot)
        return;
    end

    P1 =zeros(N1,num_1,3);P2 =zeros(N1+N2,num_2,3);
    

    for i=1:N1
       for j = 1 : num_1
           P1(i,j,:) = (P_ip+R_ip*( p(i,:) + Q1(:,j)'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)])')';
       end       
    end
    for i=1:N1+N2
       for j = 1 : num_2
           P2(i,j,:) = (P_ip+R_ip*( p(i,:) + Q2(:,j)'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)])')';
       end 
    end
%     P1 = P1*1e3;P2 = P2*1e3;
    for j = 1: num_1
        plot3(P1(1:end-1,j,1),P1(1:end-1,j,2),P1(1:end-1,j,3),'LineWidth', 2,'Color',[0.6314    0.5961    0.7647]);
    end
    for j = 1: num_2
        plot3(P2(1:end-1,j,1),P2(1:end-1,j,2),P2(1:end-1,j,3),'LineWidth', 0.1,'Color',[0.6863    0.8510    0.9098]);
    end
    
    % 最大应变计算
%     u0 = sqrt(u(1,:).^2 +u(2,:).^2 );
%     [~,up1] = max(u0(1:N1)); 
%     [~,up2] = max(u0);
%     uP1 = reshape(P1(up1,:,:),[num_1 3]);
%     uP11 = reshape(P1(up1+1,:,:),[num_1 3]);
%     plotMaxStrain(uP1,uP11,[0 0 1]);
%     uP2 = reshape(P2(up2,:,:),[num_2 3]);
%     uP21 = reshape(P2(up2+1,:,:),[num_2 3]);
%     plotMaxStrain(uP2,uP21,[1,0,0]);

    % 画间隔片和刚性段
    step_2 = round(arm_serial.assembly_para(8)*(N2-2)/arm_serial.size_para(3));
    if(psi(1)<arm_serial.size_para(1))
        ls = 0;
        L1 = psi(1);
    else
        L1 = arm_serial.size_para(1);
        ls = psi(1) - L1;
    end
    
    step_1 = round(arm_serial.assembly_para(8)*(N1-2)/L1);

    radius_Lr = arm_serial.assembly_para(2)/2;
    if(ls>0)
        for i = N1:-step_1:2
            plotcylinder(p(i,:),p(i-1,:),[ 0.9569    0.6784    0.6941],radius_Lr,0.6);
        end

    end
    if(L1>0)
        for i = (N1-step_1):-step_1:2
            pi1 = P_ip+R_ip*p(i,:)';
            pi2 = P_ip+R_ip*p(i-1,:)';

            plotcylinder(pi1,pi2,[ 0.9569    0.6784    0.6941],radius_Lr,0.6);
        end
    end

    % 第二刚性段
    pi1 = P_ip+R_ip*p(N1-1,:)';
    pi2 = P_ip+R_ip*p(N1,:)';
    plotcylinder(pi1,pi2,[0.4 0.4 0.4],radius_Lr,0.6);
    for i = (N2+N1-1):-step_2:N1+2
        pi1 = P_ip+R_ip*p(i,:)';
        pi2 = P_ip+R_ip*p(i-1,:)';
        plotcylinder(pi1,pi2,[0.7451    0.8392    0.5451],radius_Lr,0.5);
    end

    % 画末端执行器，高级版
    if(max(size(psi)) == 6)
        beta = 0;
    else
        beta = psi(7);
    end

    T=[R_ip*[Rs(N2+N1-1,1:3);Rs(N2+N1-1,4:6);Rs(N2+N1-1,7:9)]' (P_ip+R_ip*p(N2+N1-1,:)');[0 0 0 1]];
    plotEffector_keith(T, arm_serial.effector, beta, [0.5 0.5 0.5]);
    T1=[R_ip*[Rs(N2+N1,1:3);Rs(N2+N1,4:6);Rs(N2+N1,7:9)]' (P_ip+R_ip*p(N2+N1,:)');[0 0 0 1]];
    plotCoord(init_pose,1);
    plotCoord(T1,1);
end

%% 画间隔片和刚性段
function plotcylinder(u1,u2,color,r,alpha)
% 根据空间两点画圆柱
% u1,u2 ——空间两点
% color ——颜色
% r     ——半径
% alpha ——透明度
u1 = reshape(u1, [3 1]);
u2 = reshape(u2, [3 1]);
n=u2-u1;
theta=(0:2*pi/100:2*pi)'; %theta角从0到2*pi
a=cross(n,[1 0 0]);       %n与i叉乘，求取a向量
if ~any(a) %如果a为零向量，将n与j叉乘   %any检测矩阵中是否有非零元素，如果有，则返回1，否则，返回0
    a=cross(n,[0 1 0]);
end
b=cross(n,a); %求取b向量
a=a/norm(a);  %单位化a向量
b=b/norm(b);  %单位化b向量
%a,b是满足既垂直于n，又互相垂直的任意单位向量

%底面圆方程
c1=u1(1)*ones(size(theta,1),1);
c2=u1(2)*ones(size(theta,1),1);
c3=u1(3)*ones(size(theta,1),1);
x1=c1+r*a(1)*cos(theta)+r*b(1)*sin(theta); %圆上各点的x坐标
y1=c2+r*a(2)*cos(theta)+r*b(2)*sin(theta); %圆上各点的y坐标
z1=c3+r*a(3)*cos(theta)+r*b(3)*sin(theta); %圆上各点的z坐标

%顶面圆方程
c1=u2(1)*ones(size(theta,1),1);
c2=u2(2)*ones(size(theta,1),1);
c3=u2(3)*ones(size(theta,1),1);
x2=c1+r*a(1)*cos(theta)+r*b(1)*sin(theta); %圆上各点的x坐标
y2=c2+r*a(2)*cos(theta)+r*b(2)*sin(theta); %圆上各点的y坐标
z2=c3+r*a(3)*cos(theta)+r*b(3)*sin(theta); %圆上各点的z坐标

X(1,:)=x1(:);
X(2,:)=x2(:);
Y(1,:)=y1(:);
Y(2,:)=y2(:);
Z(1,:)=z1(:);
Z(2,:)=z2(:);

fill3(X(1,:),Y(1,:),Z(1,:),color,'EdgeColor','none','FaceAlpha',alpha) %底面
hold on
fill3(X(2,:),Y(2,:),Z(2,:),color,'EdgeColor','none','FaceAlpha',alpha) %顶面
hold on
surf(X,Y,Z,'facecolor',color,'edgecolor','none','FaceAlpha',alpha)    %圆柱表面
end

%% 画末端执行器，高级版
function plotEffector_keith(T, effector, beta, color)
%   this is a function to plot the gripper
%   there is two input, where the former is used to set the pose of gripper,
%   and the latter is set the open angle.
%
%   Author Keith W.
%   Ver. 1.0
%   Date 04.29.2022

if nargin == 3
    beta = 0;
end
if nargin == 2
    color = [0.5 0.5 0.5];
    beta = 0;
end
R=T(1:3,1:3);
P=T(1:3,4);
stator = effector.stator;
rotor = effector.rotor;
block_size_rotor = size(rotor,2);
block_size_stator = size(stator,2);
if(rotor(end,end) == 2)
    beta = beta/2;
    rotor2=rotor;
elseif(rotor(end,end) == 1)
    
end
r0 = eul2rotm([0 0 beta]);
for i = 1:3
    for j = 1:block_size_stator
        p1=[stator(i,j);stator(i+3,j);stator(i+6,j)];
        p1=R*p1;
        stator(i,j)=p1(1);stator(i+3,j)=p1(2);stator(i+6,j)=p1(3);
    end
    for j = 1:(block_size_rotor-1)
        p11=[rotor(i,j);rotor(i+3,j);rotor(i+6,j)];
        p1=R*r0*p11;
        rotor(i,j)=p1(1);rotor(i+3,j)=p1(2);rotor(i+6,j)=p1(3);
        if(rotor(end,end) == 2)
            p2 = R * r0'*(p11.*[1; -1; 1]);
            rotor2(i,j)=p2(1);rotor2(i+3,j)=p2(2);rotor2(i+6,j)=p2(3);
        end
    end
end
% P=[0 0 0]';
stator_x=stator(1:3,:)+P(1);stator_y=stator(4:6,:)+P(2);stator_z=stator(7:9,:)+P(3);
P=P+R*rotor(1:3,end); 
% P=[0 0 0]';
% figure;hold on;
rotor_x=rotor(1:3,1:end-1)+P(1);rotor_y=rotor(4:6,1:end-1)+P(2);rotor_z=rotor(7:9,1:end-1)+P(3);
patch(stator_x,stator_y,stator_z,'w','FaceAlpha',0.7,'EdgeColor','none','FaceColor',color);
patch(rotor_x,rotor_y,rotor_z,'w','FaceAlpha',0.7,'EdgeColor','none','FaceColor',color+0.3);
if(rotor(end,end) == 2)
    rotor_x=rotor2(1:3,1:end-1)+P(1);rotor_y=rotor2(4:6,1:end-1)+P(2);rotor_z=rotor2(7:9,1:end-1)+P(3);
    patch(rotor_x,rotor_y,rotor_z,'w','FaceAlpha',0.7,'EdgeColor','none','FaceColor',color+0.3);
end
end

%% 画坐标系
function Pr=plotCoord(T,length)
if nargin==1
    length=1;
end
    arrowLen=length*10;
    lineWidth=length;
    R=T(1:3,1:3);
    P=T(1:3,4);
    px = R*[arrowLen 0 0]'+P;
    py = R*[0 arrowLen 0]'+P;
    pz = R*[0 0 arrowLen]'+P;
    line([P(1) px(1)],[P(2) px(2)],[P(3) px(3)],'Color','r','LineStyle','-','LineWidth',lineWidth);
    line([P(1) py(1)],[P(2) py(2)],[P(3) py(3)], 'Color','g','LineStyle','-','LineWidth',lineWidth);
    line([P(1) pz(1)],[P(2) pz(2)],[P(3) pz(3)], 'Color','b','LineStyle','-','LineWidth',lineWidth);
%     plot3(P(1),P(2),P(3),'c*');
    Pr=[P px P py P pz];
end

%% 应力集中
function plotMaxStrain(P1,P2,color)
    P = P1-P2;
    dis = sqrt(P(:,1).^2 + P(:,2).^2 +P(:,3).^2);
    [~,n] = min(dis);
    plot3(P1(n,1),P1(n,2),P1(n,3),"o",LineWidth=3,Color=color);
end

