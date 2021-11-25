clc;clear;
%% ===<<Bottom-to-Up Cosserat model, Two Segments>>=== %%
%%====================List of funs and vars====================%%
% (i)Functions:
% shootingOpt-shooting method nested in an optimization framework
% estShooting-guess-residue shooting shell for force estimation kinematics
% odeCosserat1-(external) universal cosserat integration kernal
% calcCurvature- calculate the curvature at every point given the curve
%model for unloaded shape.
% (ii)Variables:
% MP-mechanical parameter struct of the robot
% FBG-model parameters about the FBG sensor
% qa-actuation lengths (phi and d can be included)
% Fe, Me-external tip load (for ref)
% fe, le-distributed load (for ref)
%%-------------------------------------------------------------%%
%%=========================More Info===========================%%
% average computing time=0.2787s


%% ===Input Specifications=== %%
e1=[1 0 0]';e2=[0 1 0]';e3=[0 0 1]';
%---mechanical parameters
MP.E=80e9;%Rod Young's modules
MP.mu=0.33;%Rod Poisson rate
MP.d1=0.50e-3;%Rod diameters
MP.d2=0.40e-3;
MP.rho=0.85e-3;%Rod pitch circle radius
MP.L=500e-3;%Robot stem length
MP.L1=37.5e-3;%Robot seg1 length
MP.L2=25e-3;%Robot seg2 length
MP.Lr=2.5e-3;%Robot rigid seg length
MP.Lg=5e-3;%Robot gipper length
MP.r11=[1 0 0]'*MP.rho;MP.r12=[0 1 0]'*MP.rho;
MP.r21=[1/sqrt(2) 1/sqrt(2) 0]'*MP.rho;MP.r22=[-1/sqrt(2) 1/sqrt(2) 0]'*MP.rho;
MP.Q1=[S(MP.r11)*e3 S(MP.r12)*e3 S(MP.r21)*e3 S(MP.r22)*e3];
MP.Q2=[S(MP.r21)*e3 S(MP.r22)*e3];
MP.I1=pi*MP.d1^4/64;MP.A1=pi*MP.d1^2/4;
MP.I2=pi*MP.d2^4/64;MP.A2=pi*MP.d2^2/4;
MP.G=MP.E/2/(1+MP.mu);MP.J=2*MP.I1;
MP.Ke1=diag([3e8 3e8 MP.A1*MP.E]);MP.Kb1=diag([MP.E*MP.I1 MP.E*MP.I1 10]);
MP.Ke2=diag([3e8 3e8 MP.A2*MP.E]);MP.Kb2=diag([MP.E*MP.I2 MP.E*MP.I2 10]);
%---fbg parameters
FBG.num=7;
FBG.s=zeros(FBG.num,1);
FBG.tip_offset=7.5e-3;
for i=FBG.num:-1:1
    FBG.s(i)=MP.L1+MP.Lr+MP.L2+MP.Lg+FBG.tip_offset-(15e-3)-(FBG.num-i)*10e-3;
end
% [delta_comp]=fbgCalibrate;
% u=strain2curvature(delta_comp);
 
 %curvatures=importdata('./0924/curvatures2.txt');
 %u=[curvatures(idx,1:7);curvatures(idx,8:14);curvatures(idx,15:21)]*1.05;
 %qa=calcActuation([0 0 30/180*pi pi/2 50/180*pi -pi/2]',MP);%psi from actual delta definition
 %qa=[0 0 qa']';
load('./0923/config2/config2.mat')
u=config2.u;
qa=config2.qa;

u=u(:,(18-FBG.num+1):18)*1.0;%use the last 7 FBG sensors

%---test of analytical estimation
u_fit=fitCurvature(u,FBG);
F_est1=tipForceEst(u_fit,MP);

% qa=[0 0     ...
%    -0.3     ...
%    -0.55*pi	...
%     0.91*pi	...
%    -0.39*pi ]'*MP.rho;
%     %inplace rot qa
%     qa=[0 0.03 -0.0020 0.0004 -0.0005 -0.0011]';
%---Reference external load; distributed not active in this version
Fe=[0 0 0]';Me=[0 0 0]';
fe=[0 0 0]';le=[0 0 0]';

%% ====Execute=== %%

[F_est,t1,t2,y1,y2]=shootingOpt(qa,u,Fe,Me,fe,le,MP,FBG);

size1=size(y1(:,1:3));size2=size(y2(:,1:3));
n1=size1(1);n2=size2(1);
figure(1);%cla;
[hAx]=plotAxis(0.02,eye(4));
[hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP,[0.08 0.17 0.55]);
plotArrow(y2(n2,1:3),F_est,norm(F_est)*4e-2,[1 0 0]);
%[hAx]=plotAxis(0.005,[y1(n1,4:6)' y1(n1,7:9)' y1(n1,10:12)' y1(n1,1:3)'; 0 0 0 1]);
%[hAx]=plotAxis(0.005,[y2(n2,4:6)' y2(n2,7:9)' y2(n2,10:12)' y2(n2,1:3)'; 0 0 0 1]);
disp(['F_est  = ' num2str(F_est(1)) ' ' num2str(F_est(2)) ' ' num2str(F_est(3))]);

%---calculate u-F matrix
%qa=zeros(6,1);
% u1=[-8.8631   -5.9792   -3.1115   -0.2569  -10.0599   -0.3712    9.2986;
%    -0.0000   -0.0000   -0.0000   -0.0000    0.0000    0.0000    0.0000;
%     0.0025    0.0123    0.0222    0.0321    0.0425    0.0525    0.0625];
% u2=[5.0217    7.9093   10.7785   13.5947  -24.3241  -14.5775   -4.6435;
%     0.0000    0.0000    0.0000    0.0000   -0.0000   -0.0000   -0.0000;
%     0.0025    0.0123    0.0222    0.0321    0.0425    0.0525    0.0625];
% u3=[-21.5471  -18.6980  -15.9836  -13.4613  -26.2864  -19.6777  -14.6753;
%    -0.0000   -0.0000   -0.0000   -0.0000    0.0000    0.0000    0.0000;
%     0.0025    0.0123    0.0222    0.0321    0.0425    0.0525    0.0625];
% u4=[-7.2578   -5.2530   -2.9939   -0.5407   -9.9546   -0.8680    8.6962
%   -14.1334  -13.7363  -13.5828  -13.6111  -19.8007  -20.3300  -21.4832
%     0.0025    0.0123    0.0222    0.0321    0.0425    0.0525    0.0625];
% u5=[-8.3678   -5.9011   -3.1781   -0.2517   -9.5409   -0.5201    9.0209;
%    13.8943   13.6799   13.6726   13.7794  -21.1433  -20.6689  -20.8869;
%     0.0025    0.0123    0.0222    0.0321    0.0425    0.0525    0.0625];
%[F_est,t1,t2,y1,y2]=shootingOpt(qa,u1,Fe,Me,fe,le,MP,FBG);
%Jacob=zeros(3,14);du=1e-1;
% for i=1:14
% dU=zeros(3,7);
% if(i<=7)
%     dU(1,i)=du;
% else
%     dU(2,i-7)=du;
% end
% [F_est_,t1,t2,y1,y2]=shootingOpt(qa,u1+dU,Fe,Me,fe,le,MP,FBG);
% Jacob(:,i)=(F_est_-F_est)/du;
% end
disp("finish")



%% ===Model Functions=== %%
function [F_est,t1,t2,y1,y2]=shootingOpt(qa,u,Fe,Me,fe,le,MP,FBG)
%num=9;
num=10;
dGuess=eye(num)*1e-5;
lambda=5e-9;
eps=1e-4;
plotFlag=0;

nc=[0 0 0]';mc=[0 0 0]';v=[0 0 0 0]';
%Guess=[nc(1:2);mc;v];
Guess=[nc(1:3);mc;v];

[Rsd,t1,y1,t2,y2]=estShooting(Guess,qa,u,Fe,Me,fe,le,MP,FBG);

if(plotFlag == 1)
    size1=size(y1(:,1:3));size2=size(y2(:,1:3));
    n1=size1(1);n2=size2(1);
    figure(1);%cla;
    [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP);
end

while(norm(Rsd)>eps)
    disp(['residue0 = ' num2str(norm(Rsd))]);

    for i=1:num
        [Rsd_]=estShooting(Guess+dGuess(:,i),qa,u,Fe,Me,fe,le,MP,FBG);
        J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
    end
    weight=0.07-FBG.s;
    weight=weight/norm(weight)*3;
    Weight=eye(21);
%     for k=1:7
%         Weight(2*k-1,2*k-1)=weight(k);
%         Weight(2*k,2*k)=weight(k);
%     end
    Guess=Guess-inv(J'*J+lambda*eye(num))*J'*(Weight*Rsd);
    [Rsd_,t1,y1,t2,y2]=estShooting(Guess,qa,u,Fe,Me,fe,le,MP,FBG);
    
    if(norm(Rsd_-Rsd)/norm(Rsd)<1e-2)%steady
        Rsd=Rsd_;
        break;
    else
        Rsd=Rsd_;
    end
    if(plotFlag==1)
        pause(0.02);
        delete(hShape);
    	size1=size(y1(:,1:3));size2=size(y2(:,1:3));
        n1=size1(1);n2=size2(1);
        [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1,n2],MP);
    end
end
%F_est = [Guess(1:2);0];
F_est=Guess(1:3);
end

function [Rsd,t1,y1,t2,y2]=estShooting(Guess,qa,u,Fe,Me,fe,le,MP,FBG)
if(length(qa)==4)
    p0=zeros(3,1);R0=eye(3);
else
    p0=[0 0 qa(2)]';R0=Expm([0 0 qa(1)]');
end

%nc=[Guess(1:2);0];
%mc=Guess(3:5);
%v=zeros(3,4);v(end,:)=Guess(6:9)';
nc=[Guess(1:3)];
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

curvFactor=1e-5;
Rsd=zeros(FBG.num*2+4+3,1);
for i=1:4
	j=FBG.s(i)/step+1;
    if(j<ceil(j))
        jc=ceil(j);jf=floor(j);
        R_c=[y1(jc,4:6)' y1(jc,7:9)' y1(jc,10:12)'];
        m_c=y1(jc,16:18)';
        R_f=[y1(jf,4:6)' y1(jf,7:9)' y1(jf,10:12)'];
        m_f=y1(jf,16:18)';
        u_c=inv(4*MP.Kb1+4*MP.Kb2)*R_c'*m_c;
        u_f=inv(4*MP.Kb1+4*MP.Kb2)*R_f'*m_f;
        u_guess(:,i)=(u_c+u_f)/2;
    else
        R=[y1(j,4:6)' y1(j,7:9)' y1(j,10:12)'];
        m=y1(j,16:18)';
        u_guess(:,i)=inv(4*MP.Kb1+4*MP.Kb2)*R'*m;
    end
    Rsd((i-1)*2+1:i*2)=(u(1:2,i)-u_guess(1:2,i))*curvFactor;
end
for i=5:FBG.num
    j=FBG.s(i)/step-39;
    if(j<ceil(j))
        jc=ceil(j);jf=floor(j);
        R_c=[y2(jc,4:6)' y2(jc,7:9)' y2(jc,10:12)'];
        m_c=y2(jc,16:18)';
        R_f=[y2(jf,4:6)' y2(jf,7:9)' y2(jf,10:12)'];
        m_f=y2(jf,16:18)';
        u_c=inv(4*MP.Kb2)*R_c'*m_c;
        u_f=inv(4*MP.Kb2)*R_f'*m_f;
        u_guess(:,i)=(u_c+u_f)/2;
    else
        R=[y2(j,4:6)' y2(j,7:9)' y2(j,10:12)'];
        m=y2(j,16:18)';
        u_guess(:,i)=inv(4*MP.Kb2)*R'*m;
    end
        Rsd((i-1)*2+1:i*2)=(u(1:2,i)-u_guess(1:2,i))*curvFactor;
end
Rsd(2*FBG.num+1:2*FBG.num+2)=yL1(19:20)-(qa(3:4)+qe1);
Rsd(2*FBG.num+3:2*FBG.num+4)=yL2(19:20)-(qa(5:6)+qe2);
Rsd(2*FBG.num+5:2*FBG.num+7)=Me-yL2(16:18)-2*( cross(R2*MP.r21,R2*MP.Ke2*v(:,3))+cross(R2*MP.r22,R2*MP.Ke2*v(:,4)));
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
    n=y_all(i,13:15)';
    m=y_all(i,16:18)';
    u(:,i)=inv(4*MP.Kb1+4*MP.Kb2)*R'*m+u_;
    u_n(i)=norm(u(:,i));
    u(3,i)=t1(i);
end
for i =n1+1:n1+n2
    R=[y_all(i,4:6)' y_all(i,7:9)' y_all(i,10:12)'];
    n=y_all(i,13:15)';
    m=y_all(i,16:18)';
    u(:,i)=inv(4*MP.Kb2)*R'*m+u_;
    u_n(i)=norm(u(:,i));
    u(3,i)=t2(i-n1);
end
% figure(3);hold on;
% plot(u_n,'-r')

end

%% ========================Basic Info========================= %%
% Force estimation model for multi-backbone continuum robots.  
% By YuyangChen
% Date 20200920
% Ver e1.0
%%-----------------------------------------------------------------------%%