clc;clear;cla
%% ===<<Bottom-to-Up Cosserat model, Two Segments>>=== %%
%%====================List of funs and vars====================%%
% (i)Functions:
% shootingOpt1- shooting method nested in an optimization framework,
%starting from the robot base (9 DoFs)
% shootingOpt2- shooting method nested in an optimization framework, 
%starting from the robot tip (4 DoFs) 
% invShooting1- guess-residue shooting shell for inverse kinematics,
%starting from the robot base
% invShooting2- guess-residue shooting shell for inverse kinematics, 
%starting from the robot tip
% odeCosserat1-(external) universal cosserat integration kernal from base
% odeCosserat2-(external) universal cosserat integration kernal from tip
% plotShape, plotAxis, drawCircle-plot funs
% (ii)Variables:
% MP-mechanical parameter struct of the robot
% Tg-target pose homogeneous transformation matrix
% Fe, Me-external tip load
% fe, le-distributed load
%%-------------------------------------------------------------%%

%% ===Input Specifications=== %%
e1=[1 0 0]';e2=[0 1 0]';e3=[0 0 1]';

MP.E=40e9;%Rod Young's modules
MP.mu=0.33;%Rod Poisson rate
MP.d1=0.50e-3;%Rod diameters
MP.d2=0.40e-3;
MP.rho=0.85e-3;%Rod pitch circle radius
MP.L=500e-3;%Robot stem length
MP.L1=38e-3;%Robot seg1 length
MP.L2=25e-3;%Robot seg2 length
MP.Lr=2e-3;%Robot rigid seg length
MP.Lg=5.0e-3;%Robot gipper length
MP.r11=[1 0 0]'*MP.rho;MP.r12=[0 1 0]'*MP.rho;
MP.r21=[1/sqrt(2) 1/sqrt(2) 0]'*MP.rho;MP.r22=[-1/sqrt(2) 1/sqrt(2) 0]'*MP.rho;
MP.Q1=[S(MP.r11)*e3 S(MP.r12)*e3 S(MP.r21)*e3 S(MP.r22)*e3];
MP.Q2=[S(MP.r21)*e3 S(MP.r22)*e3];
MP.I1=pi*MP.d1^4/64;MP.A1=pi*MP.d1^2/4;
MP.I2=pi*MP.d2^4/64;MP.A2=pi*MP.d2^2/4;
MP.G=MP.E/2/(1+MP.mu);MP.J=2*MP.I1;
MP.Ke1=diag([3e8 3e8 MP.A1*MP.E]);MP.Kb1=diag([MP.E*MP.I1 MP.E*MP.I1 10]);
MP.Ke2=diag([3e8 3e8 MP.A2*MP.E]);MP.Kb2=diag([MP.E*MP.I2 MP.E*MP.I2 10]);
MP.ell=0;
%random target generating
% pg=[(rand(2,1)-[0.5 0.5]')/15; rand*0.01+0.08];Rg=Expm(1.0*(rand(3,1)-0.5*ones(3,1)));
% Tg=[Rg pg;0 0 0 1];
%sequence target generating
Rg=eye(3);
for i=1:151
    pg(:,i)=[(i-1)*0.2e-3 0 0.09]';
end
for i=152:301
    pg(:,i)=[30e-3 (i-151)*0.2e-3 0.09]';
end
for i=302:601
    pg(:,i)=[0.03-(i-301)*0.2e-3 30e-3 0.09]';
end
for i=602:751
    pg(:,i)=[-0.03 0.03-(i-601)*0.2e-3 0.09]';
end
for i=752:1101
    pg(:,i)=[-0.03+(i-751)*0.2e-3 0 0.09]';
end



Fe=[0 0 0]';Me=[0 0 0]';
fe=[0 0 0]';le=[0 0 0]';%distributed not active in this version
QA=zeros(6,1);Res=zeros(5,1);
Guess=zeros(5,1);Guess(2)=-0.001;

for j=1:1101
Tg=[Rg pg(:,j);0 0 0 1];
if(j>1)
  Res(1:3) = pg(:,j)-pg(:,j-1);
end   

%% ====Execute=== %%
disp(['Target ' num2str(j) ': ' num2str(Tg(1,4)) ' ' num2str(Tg(2,4)) ' ' num2str(Tg(3,4)) ' ']);

[QA,Guess,Res,t1,t2,y1,y2]=shootingOpt2_SingleIK(Tg,Guess,Fe,Me,fe,le,MP);

disp(['Actuation: ' num2str(QA(1)) ' ' num2str(QA(2)) ' ' num2str(QA(3)) ' ' ...
      num2str(QA(4)) ' ' num2str(QA(5)) ' ' num2str(QA(6))]);

end

%% ===Model Functions=== %%
%---9DoF integrating
function [QA,t1,t2,y1,y2]=shootingOpt1(Tg,Fe,Me,fe,le,MP)
lambda=5e-6;
eps=1e-5;
plotFlag=0;
phi=0;d=0;

nc=Fe;mc=[0 0 0]';v=[0 0 0 0]';Guess=[phi;d;mc;v];N_ukn=length(Guess);
dGuess=eye(N_ukn)*1e-5;
[Rsd,t1,y1,t2,y2,QA]=invShooting1(Guess,Tg,Fe,Me,fe,le,MP);

if(plotFlag == 1)
    size1=size(y1(:,1:3));size2=size(y2(:,1:3));
    n1=size1(1);n2=size2(1);
    figure(1);%cla;
    [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP);
end
while(norm(Rsd)>eps)
    %disp(['residue0 = ' num2str(norm(Rsd))]);

    for i=1:N_ukn%Jacobian(DoF_rsd,DoF_guess)
        [Rsd_]=invShooting1(Guess+dGuess(:,i),Tg,Fe,Me,fe,le,MP);
        J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
    end

    Guess=Guess-inv(J'*J+lambda*eye(N_ukn))*J'*(Rsd);
    [Rsd,t1,y1,t2,y2,QA]=invShooting1(Guess,Tg,Fe,Me,fe,le,MP);
    pause(0.02);    
    
    if(plotFlag==1)
        delete(hShape);
    	size1=size(y1(:,1:3));size2=size(y2(:,1:3));
        n1=size1(1);n2=size2(1);
        [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1,n2],MP);
    end
end

end

function [Rsd,t1,y1,t2,y2,QA]=invShooting1(Guess,Tg,Fe,Me,fe,le,MP)

fe=[0 0 0]';nc=Fe;%assume fe=0.
mc=Guess(3:5);
v=zeros(3,4);v(end,:)=Guess(6:9)';
R0=Expm([0 0 Guess(1)]');
y0=[0 0 Guess(2) R0(:,1)' R0(:,2)' R0(:,3)' nc' mc' 0 0 0 0]';
[t1,y1]=odeCosserat1(y0,v,1,MP,fe,le,1e-3);
R1=[y1(end,4:6);y1(end,7:9);y1(end,10:12)]';
y1(end+1,:)=y1(end,:);y1(end,1:3)=y1(end,1:3)+[0 0 1]*MP.Lr*R1';
t1(end+1)=t1(end)+MP.Lr;
yL1=y1(end,:)';

y1_=[yL1(1:3);yL1(4:15);yL1(16:18)+2*(cross(R1*MP.r11,R1*MP.Ke1*v(:,1))+cross(R1*MP.r12,R1*MP.Ke1*v(:,2)));yL1(21:22)];

[t2,y2]=odeCosserat1(y1_,v,2,MP,fe,le,1e-3);
R2=[y2(end,4:6);y2(end,7:9);y2(end,10:12)]';
y2(end+1,:)=y2(end,:);y2(end,1:3)=y2(end,1:3)+[0 0 1]*MP.Lg*R2';
t2(end+1)=t2(end)+MP.Lg;
yL2=y2(end,:)';

qe1=(MP.L+MP.L1)*[v(3,1) v(3,2)]';
qe2=(MP.L+MP.L1+MP.L2+MP.Lr)*[v(3,3) v(3,4)]';%not used currently
QA=[Guess(1:2);yL1(19:20)-qe1;yL2(19:20)-qe2];

Rsd=zeros(9,1);

Rsd(1:3)=Me-yL2(16:18)-2*( cross(R2*MP.r21,R2*MP.Ke2*v(:,3))+cross(R2*MP.r22,R2*MP.Ke2*v(:,4)));
Rsd(4:6)=Tg(1:3,4)-yL2(1:3);
Rsd(7:9)=Logm(R2'*Tg(1:3,1:3));
end

%---4DoF reverted integrating
function [QA,t1,t2,y1,y2]=shootingOpt2(Tg,Fe,Me,fe,le,MP)
lambda=1e2;
eps=1e-5;
plotFlag=1;

v=[0 0 0 0]';
Guess=[v];N_ukn=length(Guess);
dGuess=eye(N_ukn)*1e-6;
[Rsd,t1,y1,t2,y2,QA]=invShooting2(Guess,Tg,Fe,Me,fe,le,MP);

if(plotFlag == 1)
    size1=size(y1(:,1:3));size2=size(y2(:,1:3));
    n1=size1(1);n2=size2(1);
    figure(1);%cla;
    [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP);
end
while(norm(Rsd)>eps)
    disp(['residue0 = ' num2str(norm(Rsd))]);

    for i=1:N_ukn%Jacobian(DoF_rsd,DoF_guess)
        [Rsd_]=invShooting2(Guess+dGuess(:,i),Tg,Fe,Me,fe,le,MP);
        J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
    end

    Guess0=Guess-inv(J'*J+lambda*eye(N_ukn))*J'*(Rsd);
    [Rsd0,t1,y1,t2,y2,QA]=invShooting2(Guess0,Tg,Fe,Me,fe,le,MP);
    %---LM damper---%
    if(norm(Rsd0)>=norm(Rsd))
        lambda=lambda*5;
    else
        Guess=Guess0;
        Rsd=Rsd0;
        lambda=lambda/5;
    end
    %Guess=Guess0;
    %Rsd=Rsd0;
    
    pause(0.02);    
    
    if(plotFlag==1)
        delete(hShape);
    	size1=size(y1(:,1:3));size2=size(y2(:,1:3));
        n1=size1(1);n2=size2(1);
        [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1,n2],MP);
    end
end
end

function [Rsd,t1,y1,t2,y2,QA]=invShooting2(Guess,Tg,Fe,Me,fe,le,MP)
Rg=Tg(1:3,1:3);pg=Tg(1:3,4);
v=zeros(3,4);v(end,:)=Guess';
fe=[0 0 0]';nc=Fe;%assume fe=0.
mc=Me-2*( cross(Rg*MP.r21,Rg*MP.Ke2*v(:,3)) + cross(Rg*MP.r22,Rg*MP.Ke2*v(:,4)) );
step=1e-3;

yL2=[pg' Rg(:,1)' Rg(:,2)' Rg(:,3)' nc' mc' 0 0]';
yL2_=[pg'-MP.Lg*[0 0 1]*Rg' Rg(:,1)' Rg(:,2)' Rg(:,3)' nc' mc' 0 0]';
[t2,y2]=odeCosserat2(yL2_,v,2,MP,fe,le,step);
y2=[yL2';y2];
t2=[MP.Lg+MP.L2+MP.Lr+MP.L1;t2];
R1=[y2(end,4:6);y2(end,7:9);y2(end,10:12)]';

yL1=[y2(end,1:18)';0;0;y2(end,19:20)'];
yL1_=[yL1(1:3)'-MP.Lr*[0 0 1]*R1' yL1(4:15)' (yL1(16:18)-2*(cross(R1*MP.r11,R1*MP.Ke1*v(:,1))+cross(R1*MP.r12,R1*MP.Ke1*v(:,2))) )' yL1(19:22)'];

[t1,y1]=odeCosserat2(yL1_,v,1,MP,fe,le,step);
y1=[yL1';y1];
t1=[MP.Lr+MP.L1;t1];
yL0=y1(end,:)';
R0=[y1(end,4:6);y1(end,7:9);y1(end,10:12)]';
p0=y1(end,1:3)';

qe1=(MP.L+MP.L1)*[v(3,1) v(3,2)]';
qe2=(MP.L+MP.L1+MP.L2+MP.Lr)*[v(3,3) v(3,4)]';%not used currently
QA=[[0 0 1]*Logm(R0);[0 0 1]*p0;yL0(19:20)-qe1;yL0(21:22)-qe2];

Rsd=zeros(4,1);

Rsd(1:2)=[1 0 0;0 1 0]*p0; % ?
Rsd(3:4)=[1 0 0;0 1 0]*Logm(R0);
t1=fliplr(t1')';t2=fliplr(t2')';
y1=fliplr(y1')';y2=fliplr(y2')';
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

%% ===Other Function=== %%
function [u]=calcCurvature(y1,y2,t1,t2,MP)

I=pi*MP.d^4/64;A=pi*MP.d^2/4;G=MP.E/2/(1+MP.mu);Jm=2*I;
K_se1=diag([A*G A*G A*MP.E])*8;K_se1(1,1)=8e8;K_se1(2,2)=8e8;
K_bt1=diag([MP.E*I MP.E*I G*Jm])*8;K_bt1(3,3)=10;
K_se2=diag([A*G A*G A*MP.E])*4;K_se2(1,1)=4e8;K_se2(2,2)=4e8;
K_bt2=diag([MP.E*I MP.E*I G*Jm])*4;K_bt2(3,3)=5;


v_=[0 0 1]'; u_=[0 0 0]';
size1=size(y1(:,1:3));size2=size(y2(:,1:3));
n1=size1(1);n2=size2(1);
v=zeros(3,n1+n2);u=zeros(3,n1+n2);u_n=zeros(1,n1+n2);
y_all=[y1(:,1:18);y2(:,1:18)];
for i =1:n1
    R=[y_all(i,4:6)' y_all(i,7:9)' y_all(i,10:12)'];
    n=y_all(i,13:15)';
    m=y_all(i,16:18)';
    v(:,i)=inv(K_se1)*R'*n+v_;
    u(:,i)=inv(K_bt1)*R'*m+u_;
    u_n(i)=norm(u(:,i));
    u(3,i)=t1(i);
end
for i =n1+1:n1+n2
    R=[y_all(i,4:6)' y_all(i,7:9)' y_all(i,10:12)'];
    n=y_all(i,13:15)';
    m=y_all(i,16:18)';
    v(:,i)=inv(K_se2)*R'*n+v_;
    u(:,i)=inv(K_bt2)*R'*m+u_;
    u_n(i)=norm(u(:,i));
    u(3,i)=t2(i-n1);
end
% figure(3);hold on;
% plot(u_n,'-r')

end

%% ===inverse kinematics based control
function [QA,Guess,Rsd,t1,t2,y1,y2]=shootingOpt2_SingleIK(Tg,Guess,Fe,Me,fe,le,MP)
lambda=5e-2;
eps=1e-4;
plotFlag=1;

N_ukn=length(Guess);
dGuess=eye(N_ukn)*1e-6;
count = 0;

[Rsd,t1,y1,t2,y2,QA]=invShooting3(Guess,Tg,Fe,Me,fe,le,MP);
while(norm(Rsd)>eps)
    %disp(['residue0 = ' num2str(norm(Rsd))]);

    for i=1:N_ukn%Jacobian(DoF_rsd,DoF_guess)
        [Rsd_]=invShooting3(Guess+dGuess(:,i),Tg,Fe,Me,fe,le,MP);
        J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
    end

    Guess=Guess-inv(J'*J+lambda*eye(N_ukn))*J'*(Rsd);
    if(Guess<0)
        Guess = 0;
    end
    [Rsd,t1,y1,t2,y2,QA]=invShooting3(Guess,Tg,Fe,Me,fe,le,MP);
    %---LM damper---%
    %if(norm(Rsd0)>=norm(Rsd))
        %lambda=lambda*5;
    %else
        %Guess=Guess0;
        %Rsd=Rsd0;
        %lambda=lambda/5;
    %end
    %Guess=Guess0;
    %Rsd=Rsd0;
    
    count=count+1;
    if(plotFlag==1)
        figure(1);hold on;cla;
        [hAx]=plotAxis(0.02,eye(4));
        [hAx]=plotAxis(0.01,Tg);
        size1=size(y1(:,1:3));size2=size(y2(:,1:3));
        n1=size1(1);n2=size2(1);
        [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1,n2],MP);
        plotArrow(Tg(1:3,4),Fe/norm(Fe),0.02,[0 0 0.5]);
        pause(0.03); 
        
        %delete(hShape);
    end

    if(count>10)
        break;
    end
end
disp(['IK solved']);
end

%% ========================Basic Info========================= %%
% Inverse kinematics model for multi-backbone continuum robots.  
% By YuyangChen
% Date 20200604
% Ver i1.0
%%-----------------------------------------------------------------------%%