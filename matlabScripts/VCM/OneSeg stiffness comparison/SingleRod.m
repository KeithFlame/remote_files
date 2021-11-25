function [u,pe]=SingleRod(length,Me,Fe,K)

%% ===<<Bottom-to-Up Cosserat model, One Segments>>=== %%
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
e1=[1 0 0]';e2=[0 1 0]';e3=[0 0 1]';

MP.E=50e9;%Rod Young's modules
MP.mu=0.33;%Rod Poisson rate
MP.d1=0.40e-3;%Rod diameters
MP.rho=0.85e-3;%Rod pitch circle radius
MP.L=0e-3;%Robot stem length
MP.Lf=length;
MP.L1=length;%Robot seg1 length
MP.Lr=0e-3;%Robot rigid seg length
MP.Lg=1e-3;
MP.r11=[1 0 0]'*MP.rho;MP.r12=[0 1 0]'*MP.rho;
MP.Q1=[S(MP.r11)*e3 S(MP.r12)*e3];
%MP.I=2*(pi*MP.d1^4/64)+2*(pi*MP.d1^4/64+pi*MP.d1^2/4*MP.rho^2);
MP.I = 4*(pi*MP.d1^4/64);
MP.A=4*(pi*MP.d1^2/4);
MP.G=MP.E/2/(1+MP.mu);MP.J=2*MP.I;
MP.Ke=diag([MP.G*MP.A MP.G*MP.A MP.A*MP.E]);
if(nargin == 3)
    MP.Kb=diag([MP.E*MP.I MP.E*MP.I MP.G*MP.J]);
else
    MP.Kb=diag([K K 2*MP.G*MP.I]);
end

%Fe=[0 0 0]';%Me=[0 0 0]';
fe=[0 0 0]';le=[0 0 0]';%distributed not active in this version

%% ====Execute=== %%
[Guess,t1,y1]=shootingOpt(Fe,Me,fe,le,MP);
pe=y1(end,1:3)';
size1=size(y1(:,1:3));size2=1;t2=[t1(end)];y2=[y1(end,:)];
n1=size1(1);n2=size2(1);
% figure(1);%cla;
% [hAx]=plotAxis(0.02,eye(4));
% [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP,[0.08 0.17 0.55]);
% [hAx]=plotAxis(0.005,[y1(n1,4:6)' y1(n1,7:9)' y1(n1,10:12)' y1(n1,1:3)'; 0 0 0 1]);
% [hAx]=plotAxis(0.005,[y2(n2,4:6)' y2(n2,7:9)' y2(n2,10:12)' y2(n2,1:3)'; 0 0 0 1]);
% axis equal;
%---plot curvatures---%
u=calcCurvature(y1,t1,MP);
%  u1=u(:,1);u2=u(:,end);
% u_bar=[mean(u(:,3:4),2) mean(u(:,13:14),2) mean(u(:,23:24),2) mean(u(:,33:34),2) mean(u(:,43:44),2) mean(u(:,53:54),2) mean(u(:,63:64),2)];
% figure(2);hold on;grid on;cla;
% plotCurvature(u);
% plotAxis(10,eye(4));
% axis equal;
%---calculate energy---%
% E=0;
% for i=1:n1
%     E=E+0.5*u(:,i)'*(8*MP.Kb)*u(:,i)*1e-3;
% end
% E1=E;
% for i=1:n2
%     E=E+0.5*u(:,n1+i)'*(4*MP.Kb)*u(:,n1+i)*1e-3;
% end
% E12=E;
% for j=1:4
% E=E+0.5*([0 0 Guess(6+j)]*(2*MP.Ke)*[0 0 Guess(6+j)]')*(MP.L1/2+MP.L2/2+MP.L);
% end
% disp(['Energy: E1=' num2str(E1) ', E12=' num2str(E12) ', Eall=' num2str(E)]);
end
%% ===Model Functions=== %%
function [Guess,t1,y1]=shootingOpt(Fe,Me,fe,le,MP)
dGuess=eye(6)*1e-5;
lambda=5e-10;
eps=1e-5;
plotFlag=0;

nc=[0 0 0]';mc=[0 0 0]';Guess=[nc;mc];
[Rsd,t1,y1]=forShooting(Guess,Fe,Me,fe,le,MP);

if(plotFlag == 1)
    size1=size(y1(:,1:3));
    n1=size1(1);n2=1;y2=y1(end,:);
    figure(1);%cla;
    [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP);
end

while(norm(Rsd)>eps)
    disp(['residue0 = ' num2str(norm(Rsd))]);

    for i=1:6
        [Rsd_]=forShooting(Guess+dGuess(:,i),Fe,Me,fe,le,MP);
        J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
    end

    Guess=Guess-inv(J'*J+lambda*eye(6))*J'*(Rsd);
    [Rsd,t1,y1]=forShooting(Guess,Fe,Me,fe,le,MP);
        
    if(plotFlag==1)
        pause(0.02);
        delete(hShape);
    	size1=size(y1(:,1:3));
        n1=size1(1);n2=1;y2=y1(end,:);
        [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1,n2],MP);
    end
end

end

function [Rsd,t1,y1]=forShooting(Guess,Fe,Me,fe,le,MP)
p0=[0 0 0]';R0=eye(3);
nc=Guess(1:3);
mc=Guess(4:6);

y0=[p0' R0(:,1)' R0(:,2)' R0(:,3)' nc' mc']';
[t0,y0]=odeCosserat_SingleRod(y0,MP,fe,le,1e-3,[0 MP.Lf]);
yL0=y0(end,:)';
yL0_=yL0;yL0_(13:15)=yL0(13:15)-Fe;
%yL0_(16:18)=yL0(16:18)-Me;
if(MP.L1~=MP.Lf)
    [t1,y1]=odeCosserat_SingleRod(yL0_,MP,fe,le,1e-3,[MP.Lf MP.L1]);
    R1=[y1(end,4:6);y1(end,7:9);y1(end,10:12)]';
    yL1=y1(end,:)';
    y1(end+1,:)=y1(end,:);y1(end,1:3)=y1(end,1:3)+[0 0 1]*MP.Lr*R1';
    t1(end+1)=t1(end)+MP.Lr;
    t1=[t0;t1];y1=[y0;y1];

Rsd=zeros(6,1);
Rsd(1:3)=-yL1(13:15);
Rsd(4:6)=Me-yL1(16:18);
else
    t1=t0;y1=y0;
    R1=[y1(end,4:6);y1(end,7:9);y1(end,10:12)]';
    yL1=y1(end,:)';
    y1(end+1,:)=y1(end,:);y1(end,1:3)=y1(end,1:3)+[0 0 1]*MP.Lr*R1';
    t1(end+1)=t1(end)+MP.Lr;
    

Rsd=zeros(6,1);
Rsd(1:3)=Fe-yL1(13:15);
Rsd(4:6)=Me-yL1(16:18);
end

end
function [t,y]=odeCosserat_SingleRod(y0,MP,fe,le,step,Len)
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
% date 20201012
%-------------------------------------------------------------------------%
DoF=length(y0);
    N=round((Len(2)-Len(1))/step)+1;
    t=linspace(Len(1),Len(2),N)';
    
    Kb1=MP.Kb;
    Ke1=MP.Ke;

y=zeros(N,DoF);
y(1,:)=y0';
for i=1:N-1
    p=y(i,1:3)';R=[y(i,4:6);y(i,7:9);y(i,10:12)]';n=y(i,13:15)';m=y(i,16:18)';
    u=inv(Kb1)*R'*m;u(3)=0;
    
    %constant-curvature-based evolution
    theta=step*norm(u);delta=-atan2(u(2),u(1))+pi/2;
    if(theta~=0)
        p_dot=R*( step/theta*[cos(delta)*(1-cos(theta)) sin(delta)*(cos(theta)-1) sin(theta)]' );
        R_dot=[cos(delta)^2*cos(theta)+sin(delta)^2 -sin(delta)*cos(delta)*(cos(theta)-1) cos(delta)*sin(theta);...
               sin(delta)*cos(delta)*(1-cos(theta)) cos(delta)^2+cos(theta)*sin(delta)^2 -sin(delta)*sin(theta);...
              -cos(delta)*sin(theta) sin(delta)*sin(theta) cos(theta)];
    else
        p_dot=R*[0 0 step]';
        R_dot=eye(3);
    end
    p=p+p_dot;
    R=R*R_dot;

    n_dot=-fe*step;
    m_dot=-S(p_dot)*n - le*step;
    n=n+n_dot;
    m=m+m_dot;
   
    y(i+1,:)=[p;R(:,1);R(:,2);R(:,3);n;m]';
end

end
%% ===Other Functions=== %%
function [u]=calcCurvature(y1,t1,MP)
u_=[0 0 0]';
size1=size(y1(:,1:3));
n1=size1(1);
u=zeros(3,n1);u_n=zeros(1,n1);
y_all=[y1(:,1:18)];
for i =1:n1
    R=[y_all(i,4:6)' y_all(i,7:9)' y_all(i,10:12)'];
    n=y_all(i,13:15)';
    m=y_all(i,16:18)';
    u(:,i)=inv(MP.Kb)*R'*m+u_;
    u_n(i)=norm(u(:,i));
    u(3,i)=t1(i);
end

end

%% ========================Basic Info========================= %%
% Forward kinematics model for multi-backbone continuum robots.  
% By YuyangChen
% Date 20200524
% Ver f1.0
%%-----------------------------------------------------------------------%%