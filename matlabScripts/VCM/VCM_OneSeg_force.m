clc;clear;
%% ===<<Bottom-to-Up Cosserat model, One Segments>>=== %%
%%====================List of funs and vars====================%%
% (i)Functions:
% shootingOpt-shooting method nested in an optimization framework
% estShooting-guess-residue shooting shell for force estimation kinematics
% odeCosserat3-(external) universal cosserat integration kernal
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
MP.d1=0.95e-3;%Rod diameters
MP.rho=2.6e-3;%Rod pitch circle radius
MP.L=600e-3;%Robot stem length
MP.L1=100e-3;%Robot seg1 length
MP.Lr=0;
MP.Lg=20e-3;%Robot gipper length
MP.r11=[1 0 0]'*MP.rho;MP.r12=[0 1 0]'*MP.rho;
MP.Q1=[S(MP.r11)*e3 S(MP.r12)*e3];
MP.I1=pi*MP.d1^4/64;MP.A1=pi*MP.d1^2/4;
MP.G=MP.E/2/(1+MP.mu);MP.J=2*MP.I1;
MP.Ke1=diag([3e8 3e8 MP.A1*MP.E]);MP.Kb1=diag([MP.E*MP.I1 MP.E*MP.I1 10]);
%---fbg parameters
FBG.num=10;
FBG.s=zeros(FBG.num,1);
FBG.tip_offset=10e-3;
for i=FBG.num:-1:1
    FBG.s(i)=MP.L1+MP.Lg+FBG.tip_offset-(15e-3)-(FBG.num-i+2)*10e-3;
end

[delta_comp]=fbgCalibrate;
u=strain2curvature(delta_comp);

 %curvatures=importdata('./0924/curvatures2.txt');
 %u=[curvatures(idx,1:7);curvatures(idx,8:14);curvatures(idx,15:21)]*1.05;
 %qa=calcActuation([0 0 30/180*pi pi/2 50/180*pi -pi/2]',MP);%psi from actual delta definition
 qa=[0 0 -1.5 -1.5]'*1e-3;
% load('./0923/config1/config1.mat')
% u=config1.u;
% qa=config1.qa;

u=u(:,(16-FBG.num+1):16)*1.0;%use the last 10 FBG sensors

%---test of analytical estimation
u_fit=fitCurvature(u,FBG,1);
F_est1=tipForceEst(u_fit,MP,1);

%---Reference external load; distributed not active in this version
Fe=[0 0 0]';Me=[0 0 0]';
fe=[0 0 0]';le=[0 0 0]';

%% ====Execute=== %%
tstart=tic;
[F_est,t1,y1]=shootingOpt(qa,u,Fe,Me,fe,le,MP,FBG);
toc(tstart)
size1=size(y1(:,1:3));
n1=size1(1);
figure(1);%cla;
[hAx]=plotAxis(0.02,eye(4));
[hShape]=plotShape([y1(:,1:3)],[y1(:,4:12)],[n1 0],MP,[0.08 0.17 0.55]);
plotArrow(y1(n1,1:3),F_est,norm(F_est)*4e-2,[1 0 0]);
axis([-0.06 0.06 -0.06 0.06 0 0.15])
disp(['F_est  = ' num2str(F_est(1)) ' ' num2str(F_est(2)) ' ' num2str(F_est(3))]);
%disp(['F_est1 = ' num2str(F_est1(1)) ' ' num2str(F_est1(2)) ' ' num2str(F_est1(3))]);
%---plot curvatures---%
%u=calcCurvature(y1,y2,t1,t2,MP);
%u1=u(:,1);u2=u(:,end);
%Force_(idx,:)=F_est';

%% ===Model Functions=== %%
function [F_est,t1,y1]=shootingOpt(qa,u,Fe,Me,fe,le,MP,FBG)
num=8;
dGuess=eye(num)*1e-5;
lambda=5e-9;
eps=1e-4;
plotFlag=0;

nc=[0 0 0]';mc=[0 0 0]';v=[0 0]';Guess=[nc;mc;v];
%test
%Guess=[.1 .1 0 0 0 .01 .01 .01 .01]';
[Rsd,t1,y1]=estShooting(Guess,qa,u,Fe,Me,fe,le,MP,FBG);

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

    Guess=Guess-inv(J'*J+lambda*eye(num))*J'*(Rsd);
    [Rsd_,t1,y1]=estShooting(Guess,qa,u,Fe,Me,fe,le,MP,FBG);
    
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
F_est = Guess(1:3);
end

function [Rsd,t1,y1]=estShooting(Guess,qa,u,Fe,Me,fe,le,MP,FBG)
if(length(qa)==4)
    p0=zeros(3,1);R0=eye(3);
else
    p0=[0 0 qa(2)]';R0=Expm([0 0 qa(1)]');
end

nc=Guess(1:3);
mc=Guess(4:6);
v=zeros(3,2);v(end,:)=Guess(7:8)';
step=1e-3;

y0=[p0' R0(:,1)' R0(:,2)' R0(:,3)' nc' mc' 0 0]';
[t1,y1]=odeCosserat3(y0,v,MP,fe,le,step);
R1=[y1(end,4:6);y1(end,7:9);y1(end,10:12)]';
y1(end+1,:)=y1(end,:);y1(end,1:3)=y1(end,1:3)+[0 0 1]*MP.Lg*R1';
t1(end+1)=t1(end)+MP.Lg;
yL1=y1(end,:)';


qe1=(MP.L+MP.L1)*[v(3,1) v(3,2)]';

curvFactor=1e-5;
Rsd=zeros(FBG.num*2+2+3,1);
for i=1:FBG.num
	j=FBG.s(i)/step+1;
    if(j<ceil(j))
        jc=ceil(j);jf=floor(j);
        R_c=[y1(jc,4:6)' y1(jc,7:9)' y1(jc,10:12)'];
        m_c=y1(jc,16:18)';
        R_f=[y1(jf,4:6)' y1(jf,7:9)' y1(jf,10:12)'];
        m_f=y1(jf,16:18)';
        u_c=inv(4*MP.Kb1)*R_c'*m_c;
        u_f=inv(4*MP.Kb1)*R_f'*m_f;
        u_guess(:,i)=(u_c+u_f)/2;
    else
        R=[y1(j,4:6)' y1(j,7:9)' y1(j,10:12)'];
        m=y1(j,16:18)';
        u_guess(:,i)=inv(4*MP.Kb1)*R'*m;
    end
    Rsd((i-1)*2+1:i*2)=(u(1:2,i)-u_guess(1:2,i))*curvFactor;
end

Rsd(2*FBG.num+1:2*FBG.num+2)=yL1(19:20)-(qa(3:4)+qe1);
Rsd(2*FBG.num+3:2*FBG.num+5)=Me-yL1(16:18)-2*( cross(R1*MP.r11,R1*MP.Ke1*v(:,1))+cross(R1*MP.r12,R1*MP.Ke1*v(:,2)));
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