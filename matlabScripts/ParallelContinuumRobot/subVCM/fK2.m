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
% MP-mechanical parameter struct of the robot
% qa-actuation lengths (phi and d can be included)
% Fe, Me-external tip load
% fe, le-distributed load
%%-------------------------------------------------------------%%

%% ===Input Specifications=== %%
x= [100 10 25 1.5 0.1 5 1 500 0 0];
psi = [80 0 0 0 45 0];
psi(2:6) = psi(2:6)*pi/180;
u = fromPsi2Curvature(psi,x);
[qa,MP] = fC2M_2(u,x,psi(1));
qa=[0 0 qa]';
%inplace rot qa
  %qa=[0 0.03 -0.0020 0.0004 -0.0005 -0.0011]';
%qa=[0 10 -0.736431 0 -0.314704 0.314704]'*1e-3;
Fe=[0.7 -0. 0.]';Me=[0 0 0]';
fe=[0 0 0]';le=[0 0 0]';%distributed not active in this version

%% ====Execute=== %%
[Guess,t1,t2,y1,y2]=shootingOpt(qa,Fe,Me,fe,le,MP);
size1=size(y1(:,1:3));size2=size(y2(:,1:3));
n1=size1(1);n2=size2(1);
figure(1);
[~]=plotAxis(0.01,eye(4));
[hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP,[0.08 0.17 0.55]);
[~]=plotAxis(0.005,[y1(n1,4:6)' y1(n1,7:9)' y1(n1,10:12)' y1(n1,1:3)'; 0 0 0 1]);
[hAx]=plotAxis(0.005,[y2(n2,4:6)' y2(n2,7:9)' y2(n2,10:12)' y2(n2,1:3)'; 0 0 0 1]);

p=y2(end,1:3)';R=[y2(end,4:6);y2(end,7:9);y2(end,10:12)]';T=[R p;0 0 0 1];
[F_est,t1est,t2est,y1est,y2est]=shootingOptEst(qa(3:6),T,Guess,fe,le,MP);

%---plot curvatures---%
u=calcCurvature(y1,y2,t1,t2,MP);
u1=u(:,1);u2=u(:,end);

block_size = MP.L1*1000;
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

%% ===Model Functions=== %%
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
qe2=(MP.L+MP.L1+MP.L2+MP.Lr)*[v(3,3) v(3,4)]'; % elongation seg2

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
qe2=(MP.L+MP.L1+MP.L2+MP.Lr)*[v(3,3) v(3,4)]';

Rsd=zeros(10,1);
 % boundary conditions
Rsd(1:3)=yL2(1:3,:)-pg; % end pose
Rsd(4:6)=Logm(R2'*Rg);
Rsd(7:8) =yL1(19:20)-(qa(1:2)+qe1); % path length
Rsd(9:10)=yL2(19:20)-(qa(3:4)+qe2);
end

%% ===Other Functions=== %%
function [u]=calcCurvature(y1,y2,t1,t2,MP)
u_=[0 0 0]';
size1=size(y1(:,1:3));size2=size(y2(:,1:3));
n1=size1(1);n2=size2(1);
u=zeros(3,n1+n2);u_n=zeros(1,n1+n2);
y_all=[y1(:,1:18);y2(:,1:18)];
for i =1:n1
    R=[y_all(i,4:6)' y_all(i,7:9)' y_all(i,10:12)'];
    m=y_all(i,16:18)';
    u(:,i)=eye(3)/(4*MP.Kb1+4*MP.Kb2)*R'*m+u_;
    u_n(i)=norm(u(:,i));
    u(3,i)=t1(i);
end
for i =n1+1:n1+n2
    R=[y_all(i,4:6)' y_all(i,7:9)' y_all(i,10:12)'];
    m=y_all(i,16:18)';
    u(:,i)=eye(3)/(4*MP.Kb2)*R'*m+u_;
    u_n(i)=norm(u(:,i));
    u(3,i)=t2(i-n1);
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
if(SegIdx == 1)
    Q=MP.Q1;
    N=round(MP.L1/step)+1;
    t=linspace(0,MP.L1,N)';
    mod_SegIdx=1;
    Kb = 4*Kb1+4*Kb2;
    step=MP.L1/(N-1);
elseif(SegIdx == 2)
    Q=MP.Q2;
    N=round(MP.L2/step)+1;
    t=linspace(MP.L1+MP.Lr,MP.L1+MP.Lr+MP.L2,N)';
    mod_SegIdx=0;
    Kb = 4*Kb2;
    step=MP.L2/(N-1);
elseif(SegIdx == 0)
    Q=MP.Q1;
    N=round(MP.ell/step)+1;
    t=linspace(0,MP.ell,N)';
    mod_SegIdx=1;
%     gamma=4.2;
    gamma=1;
    Kb = 4*Kb1+4*Kb2 + (gamma-1)*8*Kb2;
    step=MP.ell/(N-1);
end
    
y=zeros(N,DoF);U=zeros(N,3);
y(1,:)=y0';R=[y0(4:6) y0(7:9) y0(10:12)];m=y0(16:18);
for i=1:N-1
    p=y(i,1:3)';R=[y(i,4:6);y(i,7:9);y(i,10:12)]';n=y(i,13:15)';m=y(i,16:18)';q=y(i,19:end)';
    u=Kb\R'*m;u(3)=0;
    
    theta=step*norm(u);delta=-atan2(u(2),u(1))+pi/2;
    costheta=cos(theta);sintheta=sin(theta);cosdelta=cos(delta);sindelta=sin(delta); % cache
    if(theta~=0)
        p_dot=R*( step/theta*[cosdelta*(1-costheta) sindelta*(costheta-1) sintheta]' );
%         R_dot=[cosdelta^2*costheta+sindelta^2 -sindelta*cosdelta*(costheta-1) cosdelta*sintheta;...
%                sindelta*cosdelta*(1-costheta) cosdelta^2+costheta*sindelta^2 -sindelta*sintheta;...
%               -cosdelta*sintheta sindelta*sintheta costheta];
        R_dot=[cosdelta^2*costheta+sindelta^2 sindelta*cosdelta*(costheta-1) cosdelta*sintheta;...
               -sindelta*cosdelta*(1-costheta) cosdelta^2+costheta*sindelta^2 sindelta*sintheta;...
              -cosdelta*sintheta -sindelta*sintheta costheta];
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


%% plot
function [hAx]=plotAxis(scale,Te)
%-----plot frames
% ver p1.0
% By Yuyang Chen
% Date 20200606
%-----------------------%
    a=linspace(0,scale,2);
    b=zeros(1,2);
    Re=Te(1:3,1:3);Pe=Te(1:3,4);
    X=[a;b;b];Y=[b;a;b];Z=[b;b;a];
    X=Re*X+[Pe Pe];
    Y=Re*Y+[Pe Pe];
    Z=Re*Z+[Pe Pe];
    hold on;
    hX=plot3(X(1,:),X(2,:),X(3,:),'-r','LineWidth',3);
    hY=plot3(Y(1,:),Y(2,:),Y(3,:),'-b','LineWidth',3);
    hZ=plot3(Z(1,:),Z(2,:),Z(3,:),'-','color',[0 0.65 0],'LineWidth',3);%,'Color',[0.8 0.6 0.8]);
    hAx=[hX hY hZ];
end

function [hShape]=plotShape(p,Rs,N,MP,color)
%-----plot variable curvature robot shape from continuum robots
% p(N1+N2 by 3)-central backbone positions, Rs(N1+N2 by 9)-central backbone
%orientations;
% N=[N1,N2]-number of points;
% MP-Mechanical properties;
%---------info----------%
% By Yuyang Chen
% ver p1.2
% date 20200606
%------------------------------------------------------------------------%
if(nargin == 4)
    %color=[0.8 0.4 0];
    color=[0 0 0];
end


rho1=[1 0 0]'*MP.rho1;rho2=[0 1 0]'*MP.rho1;rho3=[-1 0 0]'*MP.rho1;rho4=[0 -1 0]'*MP.rho1;
rho5=[sqrt(2)/2*1 sqrt(2)/2*1 0]'*MP.rho2;rho6=[-sqrt(2)/2*1 sqrt(2)/2*1 0]'*MP.rho2;
rho7=[-sqrt(2)/2*1 -sqrt(2)/2*1 0]'*MP.rho2;rho8=[sqrt(2)/2*1 -sqrt(2)/2*1 0]'*MP.rho2;

N1=N(1);N2=N(2);
p1=zeros(N1,3);p2=zeros(N1,3);p3=zeros(N1,3);p4=zeros(N1,3);
p5=zeros(N1+N2,3);p6=zeros(N1+N2,3);p7=zeros(N1+N2,3);p8=zeros(N1+N2,3);

for i=1:N1
   p1(i,:)=p(i,:)+rho1'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   p2(i,:)=p(i,:)+rho2'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   p3(i,:)=p(i,:)+rho3'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   p4(i,:)=p(i,:)+rho4'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   
   p5(i,:)=p(i,:)+rho5'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   p6(i,:)=p(i,:)+rho6'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   p7(i,:)=p(i,:)+rho7'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   p8(i,:)=p(i,:)+rho8'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)]; 
end
for i=N1+1:N1+N2
   p5(i,:)=p(i,:)+rho5'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   p6(i,:)=p(i,:)+rho6'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   p7(i,:)=p(i,:)+rho7'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)];
   p8(i,:)=p(i,:)+rho8'*[Rs(i,1:3);Rs(i,4:6);Rs(i,7:9)]; 
end
figure(1);hold on;
hc=plot3(p(1:end-1,1),p(1:end-1,2),p(1:end-1,3),'--','LineWidth',1,'Color',color);
%hc2=plot3(p(end-1:end,1),p(end-1:end,2),p(end-1:end,3),'-','LineWidth',5,'Color',color);
hg=drawGripper(p(end-1,:)',[Rs(end,1:3);Rs(end,4:6);Rs(end,7:9)]',MP.rho2*0.9,MP.Lg,color);
h1=plot3(p1(1:end-1,1),p1(1:end-1,2),p1(1:end-1,3),'-','LineWidth',1,'Color',color);
h2=plot3(p2(1:end-1,1),p2(1:end-1,2),p2(1:end-1,3),'-','LineWidth',1,'Color',color);
h3=plot3(p3(1:end-1,1),p3(1:end-1,2),p3(1:end-1,3),'-','LineWidth',1,'Color',color);
h4=plot3(p4(1:end-1,1),p4(1:end-1,2),p4(1:end-1,3),'-','LineWidth',1,'Color',color);
hr=drawRigid(p(N1-1,:)',[Rs(N1-1,1:3);Rs(N1-1,4:6);Rs(N1-1,7:9)]',MP.rho1*1.1,MP.Lr,color);
if(N(2)~=1)
h5=plot3(p5(1:end-1,1),p5(1:end-1,2),p5(1:end-1,3),'-','LineWidth',1,'Color',color);
h6=plot3(p6(1:end-1,1),p6(1:end-1,2),p6(1:end-1,3),'-','LineWidth',1,'Color',color);
h7=plot3(p7(1:end-1,1),p7(1:end-1,2),p7(1:end-1,3),'-','LineWidth',1,'Color',color);
h8=plot3(p8(1:end-1,1),p8(1:end-1,2),p8(1:end-1,3),'-','LineWidth',1,'Color',color);
else
    h5=[];
    h6=[];
    h7=[];
    h8=[];
end

hD1=drawCircle(p(N1-1,:),[Rs(N1-1,1:3);Rs(N1-1,4:6);Rs(N1-1,7:9)]',MP.rho1+0.15*1e-3,color);
if(N2~=0)
hD2=drawCircle(p(N1,:),[Rs(N1,1:3);Rs(N1,4:6);Rs(N1,7:9)]',MP.rho2+0.15*1e-3,color);
hD3=drawCircle(p(N1+N2-1,:),[Rs(N1+N2-1,1:3);Rs(N1+N2-1,4:6);Rs(N1+N2-1,7:9)]',MP.rho2+0.15*1e-3,color);
else
    hD2=[];
    hD3=[];
end
%hD4=drawCircle(p(N1+N2,:),[Rs(N1+N2,1:3);Rs(N1+N2,4:6);Rs(N1+N2,7:9)]',MP.rho,color);
hShape=[hc hg h1 h2 h3 h4 hr h5 h6 h7 h8 hD1 hD2 hD3]';
grid on;axis equal;
%axis([-0.05 0.05 -0.05 0.05 0 0.1]);

end

function [hG]=drawGripper(p,R,rho,Lg,color)
if(nargin == 4)
    color=[0 0 0];
end
if(Lg == 0)
    hG=[];
    return;
end
D1=rho*cos(pi/6);
D2=rho*sin(pi/6);
H1=Lg*0.3;
H2=Lg*0.4;
p0=[D1 D2 0;D1 -D2 0;-D1 -D2 0;-D1 D2 0;...
   D1 D2 H1;D1 -D2 H1;-D1 -D2 H1;-D1 D2 H1;...
   0 D2 H2;0 -D2 H2;...
   D1 D2 Lg;D1 -D2 Lg;-D1 -D2 Lg;-D1 D2 Lg]';
p1=p+R*p0;
% X1=[p1(1,1) p1(1,5) p1(1,9) p1(1,8) p1(1,4)];
% Y1=[p1(2,1) p1(2,5) p1(2,9) p1(2,8) p1(2,4)];
% Z1=[p1(3,1) p1(3,5) p1(3,9) p1(3,8) p1(3,4)];
h1=patch([p1(1,1) p1(1,5) p1(1,9) p1(1,8) p1(1,4)]',[p1(2,1) p1(2,5) p1(2,9) p1(2,8) p1(2,4)]',[p1(3,1) p1(3,5) p1(3,9) p1(3,8) p1(3,4)]',color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h2=patch([p1(1,2) p1(1,6) p1(1,10) p1(1,7) p1(1,3)],[p1(2,2) p1(2,6) p1(2,10) p1(2,7) p1(2,3)],[p1(3,2) p1(3,6) p1(3,10) p1(3,7) p1(3,3)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h3=patch([p1(1,1) p1(1,2) p1(1,6) p1(1,5)],[p1(2,1) p1(2,2) p1(2,6) p1(2,5)],[p1(3,1) p1(3,2) p1(3,6) p1(3,5)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h4=patch([p1(1,4) p1(1,3) p1(1,7) p1(1,8)],[p1(2,4) p1(2,3) p1(2,7) p1(2,8)],[p1(3,4) p1(3,3) p1(3,7) p1(3,8)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h5=patch([p1(1,5) p1(1,6) p1(1,10) p1(1,9)],[p1(2,5) p1(2,6) p1(2,10) p1(2,9)],[p1(3,5) p1(3,6) p1(3,10) p1(3,9)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h6=patch([p1(1,8) p1(1,7) p1(1,10) p1(1,9)],[p1(2,8) p1(2,7) p1(2,10) p1(2,9)],[p1(3,8) p1(3,7) p1(3,10) p1(3,9)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h7=patch([p1(1,11) p1(1,12) p1(1,10) p1(1,9)],[p1(2,11) p1(2,12) p1(2,10) p1(2,9)],[p1(3,11) p1(3,12) p1(3,10) p1(3,9)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h8=patch([p1(1,14) p1(1,13) p1(1,10) p1(1,9)],[p1(2,14) p1(2,13) p1(2,10) p1(2,9)],[p1(3,14) p1(3,13) p1(3,10) p1(3,9)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
hG=[h1 h2 h3 h4 h5 h6 h7 h8];
end

function [hR]=drawRigid(p,R,rho,Lr,color)
if(nargin==4)
    color=[0 0 0];
end
if(Lr == 0)
    hR=[];
    return;
end
N=16;%even
[x,y,z]=cylinder(rho,N);
p0=zeros(3,2*N);
for i=1:N
    if(mod(i,2)==1)
    p0(:,2*i-1)=[x(1,i) y(1,i) Lr*z(1,i)]';
    p0(:,2*i)=[x(2,i) y(2,i) Lr*z(2,i)]';
    else
    p0(:,2*i-1)=[x(2,i) y(2,i) Lr*z(2,i)]';
    p0(:,2*i)=[x(1,i) y(1,i) Lr*z(1,i)]';
    end
end
p0(:,2*N+1:2*N+2)=p0(:,1:2);
p1=p+R*p0;
hR=zeros(1,N);
for i=1:2:2*N
    h1=patch(p1(1,i:i+3),p1(2,i:i+3),p1(3,i:i+3),color,'FaceColor',color,'FaceAlpha',0.4,'EdgeAlpha',0,'EdgeColor',color,'LineWidth',1);
    hR((i+1)/2)=h1;
end

end

function [hD]=drawCircle(p,R,rho,color)
%-----plot circles (disks) with pose {p,R} and radius rho
% ver p1.0
% By Yuyang Chen
% Date 20200606
%--------------------------------------------------------%
N=20;
px = zeros(1,20);
py = zeros(1,20);
pz = zeros(1,20);
for i=1:N
    alpha=i/N*2*pi;
    px(i)=R(1,1)*rho*cos(alpha)+R(1,2)*rho*sin(alpha)+p(1);
    py(i)=R(2,1)*rho*cos(alpha)+R(2,2)*rho*sin(alpha)+p(2);
    pz(i)=R(3,1)*rho*cos(alpha)+R(3,2)*rho*sin(alpha)+p(3);
end
hD=patch(px,py,pz,color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
end

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
%% ========================Basic Info========================= %%
% Forward kinematics model for multi-backbone continuum robots.  
% By YuyangChen
% Date 20200524
% Ver f1.0
%%-----------------------------------------------------------------------%%