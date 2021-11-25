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
MP.E=55e9;%Rod Young's modules
MP.mu=0.3;%Rod Poisson rate
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
MP.G=MP.E/2/(1+MP.mu);
MP.Ke1=diag([MP.A1*MP.G MP.A1*MP.G MP.A1*MP.E]);MP.Kb1=diag([MP.E*MP.I1 MP.E*MP.I1 2*MP.G*MP.I1]);
MP.Ke2=diag([MP.A2*MP.G MP.A2*MP.G MP.A2*MP.E]);MP.Kb2=diag([MP.E*MP.I2 MP.E*MP.I2 2*MP.G*MP.I2]);
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
load('./1210/config5.mat');
u=config5.u;
u=Expm([0 0 -5/180*pi]')*u;
qa=config5.qa;
strain=config5.strain;
%u=u(:,(18-FBG.num+1):18)*1.0;%use the last 7 FBG sensors
%u=[-8.43 -10.56 -9.58 -2.77 6.00 10.608 9.783;0.48 -0.24 -0.67 0.19 1.003 0.466 0.75; 0 0 0 0 0 0 0]
%qa=[0 0 0 -1.00351 0.196875 0.196875]'*1e-3;
%---test of analytical estimation
% u_fit=fitCurvature(u,FBG);
% F_est1=tipForceEst(u_fit,MP);

% qa=[0 0     ...
%    -0.3     ...
%    -0.55*pi	...
%     0.91*pi	...
%    -0.39*pi ]'*MP.rho;
%---Reference external load; distributed not active in this version
Fe=[0 0 0]';Me=[0 0 0]';
fe=[0 0 0]';le=[0 0 0]';

%% ====Execute=== %%
EKF_force_propagate(u,qa,strain,MP,FBG);

%% ===EKF=== %%
function [] = EKF_force_propagate(u,qa,strain,MP,FBG)
%---initialize

Fe = [0 0 0]';
P = diag([1 1 0])*1e0;
Q_prc = diag([1 1 0])*1e2;
Q_obs = eye(21)*1e-5;
Me=zeros(3,1);fe=zeros(3,1);le=zeros(3,1);

figure(1);global Ori_1 Posi_1;
Ori_1=eye(3);Posi_1=[0 0 0]';
p_obs=zeros(21,1);
for i=1:7
    delta_p(i)=pi/2-atan2(u(2,i),u(1,i));
    theta_p(i)=10e-3*norm(u(:,i));
    PlotSnake(delta_p(i),theta_p(i),10e-3,[5e-4 1e3]);
    p_obs(3*i-2:3*i)=Posi_1;
end
figure(2);cla;axis on;
for i=1:10
    u_obs = [u(1:2,1);u(1:2,2);u(1:2,3);u(1:2,4);u(1:2,5);u(1:2,6);u(1:2,7)]+randn(14,1)*0.1;
    %---time update
    Fe_ = Fe;
    P_ = P + Q_prc;
    %---measurement update
    [p_guess,t1,t2,y1,y2,C]=shootingOpt(qa,Fe_,Me,fe,le,MP,FBG);
    K = P_*C'*inv( C*P_*C'+Q_obs );
    Fe = Fe_+K*(p_obs - p_guess);
    P = (eye(3)-K*C)*P_;
    %---longitudinal force calc

    Fe_longi=strain/100;
%     Fe_norm = norm(Fe)*(Ftip+sqrt( Ftip^2-5*(Ftip^2-1) ))/2/(Ftip^2-1);
%     if(norm(Fe_norm)<norm(Fe))
%         Fe_norm = norm(Fe)*(Ftip+sqrt( Ftip^2-5*(Ftip^2-1) ))/2/(Ftip^2-1);
%     end
%     Fe_longi=sqrt(Fe_norm^2-norm(Fe)^2);
%     if(Ftip<0)
%         Fe_longi=-Fe_longi;
%     end
        %---plot & disp
        size1=size(y1(:,1:3));size2=size(y2(:,1:3));
        n1=size1(1);n2=size2(1);
        figure(1);cla;
        [hAx]=plotAxis(0.02,eye(4));
        [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP,[0.08 0.17 0.55]);
        F_est = [y2(end,4:6);y2(end,7:9);y2(end,10:12)]'*[Fe(1:2);Fe_longi];%world coordinate force
        %F_est = config1.Fe;
        F_g = [0 0.4 0]';
        %F_g_longi = dot(F_g,y2(end,10:12))
        %F_g = -F_est(3)/y2(end,12)*y2(end,10:12)'+F_est;%gravity force
        plotArrow(y2(n2,1:3),F_est/60,norm(F_est)*5e-3,[1 0 0]);
        plotArrow(y2(n2,1:3),F_g/60,norm(F_g)*5e-3,[0 0 1]);
        %disp(['Fe_ = ' num2str(Fe_(1)) ' ' num2str(Fe_(2)) ' ' num2str(Fe_(3))]);
        disp(['F_est  = ' num2str(F_est(1)) ' ' num2str(F_est(2)) ' ' num2str(F_est(3))]);
        disp(['F_grv  = ' num2str(F_g(1)) ' ' num2str(F_g(2)) ' ' num2str(F_g(3))]);
        %--------------
        figure(2);hold on;
        plot(i,F_est(1),'*r');
        plot(i,F_est(2),'*g');
        plot(i,F_est(3),'*b');
end

figure(1);global Ori_1 Posi_1;
Ori_1=eye(3);Posi_1=[0 0 0]';
p_obs=zeros(21,1);
for i=1:7
    delta_p(i)=pi/2-atan2(u(2,i),u(1,i));
    theta_p(i)=10e-3*norm(u(:,i));
    PlotSnake(delta_p(i),theta_p(i),10e-3,[5e-4 1e3]);
    p_obs(3*i-2:3*i)=Posi_1;
end

end


%% ===Model Functions=== %%
function [p_guess,t1,t2,y1,y2,Cp]=shootingOpt(qa,Fe,Me,fe,le,MP,FBG)
%num=9;
num=4;
dGuess=eye(num)*1e-5;
dF=eye(3)*1e-5;
lambda=5e-9;
eps=1e-5;
plotFlag=0;

v=[0 0 0 0]';
Guess=[v];

[Rsd,t1,y1,t2,y2,pO]=estShooting(Guess,qa,Fe,Me,fe,le,MP,FBG);

%     if(norm(Fe)~=0)
%         strain_tip=1/norm(Fe)*(0.5*norm(Fe(1:2))+Fe(3));
%     else
%         strain_tip=0;
%     end
%     for i=1:3
%         Fe_=Fe+dF(:,i);
%         if(norm(Fe_)~=0)
%             strain_tip_=1/norm(Fe_)*(0.5*norm(Fe_(1:2))+Fe_(3));
%         else
%             straint_tip_=0;
%         end
%         J_strain_tip(:,i)=(strain_tip_-strain_tip)/norm(dF(:,i));
%     end

if(plotFlag == 1)
    size1=size(y1(:,1:3));size2=size(y2(:,1:3));
    n1=size1(1);n2=size2(1);
    figure(1);cla;
    [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP);
end

while(1)
    %disp(['residue0 = ' num2str(norm(Rsd(15:18)))]);
    %boundary value Jacobian
    for i=1:num
        [Rsd_,t1,y1,t2,y2,pO_]=estShooting(Guess+dGuess(:,i),qa,Fe,Me,fe,le,MP,FBG);
        J(:,i)=(Rsd_(15:18)-Rsd(15:18))/norm(dGuess(:,i));
    end
    %compliance matrix of positions
    for i=1:3
        [Rsd_,t1,y1,t2,y2,pO_]=estShooting(Guess,qa,Fe+dF(:,i),Me,fe,le,MP,FBG);
        C(:,i)=(Rsd_(1:14)-Rsd(1:14))/norm(dF(:,i));
        Cp(:,i)=(pO_-pO)/norm(dF(:,i));
    end
    %----
    if(norm(Rsd(15:18))<eps)
        break;
    end
    Guess=Guess-inv(J'*J+lambda*eye(num))*J'*(Rsd(15:18));
    [Rsd_,t1,y1,t2,y2,pO_]=estShooting(Guess,qa,[Fe(1:2);0],Me,fe,le,MP,FBG);
    
    if(norm(Rsd_(15:18)-Rsd(15:18))/norm(Rsd(15:18))<1e-2)%steady
        Rsd=Rsd_;
        pO=pO_;
        break;
    else
        Rsd=Rsd_;
        pO=pO_;
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
u_guess=Rsd(1:14);
p_guess=pO;
%u_guess=[Rsd(3:6);Rsd(11:12)];
%C=[C(3:6,:);C(11:12,:)];
end

function [Rsd,t1,Y1,t2,Y2,p_output]=estShooting(Guess,qa,Fe,Me,fe,le,MP,FBG)
%the reference frame is {g}
p2=zeros(3,1);R2=eye(3);
MP.ell = qa(2);
v=zeros(3,4);
nc=Fe;ns=[Fe(1:2);0];
for i=1:4
v(1:3,i)=inv(MP.Ke2)*R2'*ns;
v(3,i)=Guess(i);
end
mc=Me-2*cross(MP.r21,MP.Ke2*v(:,3))-2*cross(MP.r22,MP.Ke2*v(:,4));
q1=qa(3:4)+(MP.L+MP.L1)*v(end,1:2)';
q2=qa(5:6)+(MP.L+MP.L1+MP.Lr+MP.L2)*v(end,3:4)';

step=1e-3;

y2=[p2' R2(:,1)' R2(:,2)' R2(:,3)' nc' mc' q2']';
y2_=y2;y2_(1:3)=y2(1:3)-R2*[0 0 1]'*MP.Lg;
[t2,Y2]=odeCosserat_inverse(y2_,v,2,MP,fe,le,step);
R1=[Y2(end,4:6);Y2(end,7:9);Y2(end,10:12)]';
Y2(end+1,:)=zeros(1,20);
Y2(2:end,:)=Y2(1:end-1,:);
Y2(1,:)=y2';
t2(end+1)=t2(end);
t2(2:end)=t2(1:end-1);
t2(1)=MP.Lg+MP.L2+MP.L1+MP.Lr;

Y2(end+1,:)=Y2(end,:);Y2(end,1:3)=Y2(end,1:3)-[0 0 1]*MP.Lr*R1';
t2(end+1)=t2(end)-MP.Lr;
y1=Y2(end,:)';

for i=1:2
v(1:3,i)=inv(MP.Ke1)*R1'*ns;
v(3,i)=Guess(i);
end

y1_=[y1(1:3);y1(4:15);y1(16:18)-2*(cross(R1*MP.r11,R1*MP.Ke1*v(:,1))+cross(R1*MP.r12,R1*MP.Ke1*v(:,2)));q1;y1(19:20)];

[t1,Y1]=odeCosserat_inverse(y1_,v,1,MP,fe,le,step);
R0=[Y1(end,4:6);Y1(end,7:9);Y1(end,10:12)]';
y0=Y1(end,:)';
%---stem insertion is not recommended.

curvFactor=1;
Rsd=zeros(FBG.num*2+4,1);
for i=1:4
	j=FBG.s(i)/step+1;
    if(j<ceil(j))
        jc=ceil(j);jf=floor(j);
        R_c=[Y1(jc,4:6)' Y1(jc,7:9)' Y1(jc,10:12)'];
        m_c=Y1(jc,16:18)';
        R_f=[Y1(jf,4:6)' Y1(jf,7:9)' Y1(jf,10:12)'];
        m_f=Y1(jf,16:18)';
        u_c=inv(4*MP.Kb1+4*MP.Kb2)*R_c'*m_c;
        u_f=inv(4*MP.Kb1+4*MP.Kb2)*R_f'*m_f;
        u_guess(:,i)=(u_c+u_f)/2;
    else
        R=[Y1(j,4:6)' Y1(j,7:9)' Y1(j,10:12)'];
        m=Y1(j,16:18)';
        u_guess(:,i)=inv(4*MP.Kb1+4*MP.Kb2)*R'*m;
    end
    Rsd((i-1)*2+1:i*2)=u_guess(1:2,i)*curvFactor;
end
for i=5:FBG.num
    j=FBG.s(i)/step-39;
    if(j<ceil(j))
        jc=ceil(j);jf=floor(j);
        R_c=[Y2(jc,4:6)' Y2(jc,7:9)' Y2(jc,10:12)'];
        m_c=Y2(jc,16:18)';
        R_f=[Y2(jf,4:6)' Y2(jf,7:9)' Y2(jf,10:12)'];
        m_f=Y2(jf,16:18)';
        u_c=inv(4*MP.Kb2)*R_c'*m_c;
        u_f=inv(4*MP.Kb2)*R_f'*m_f;
        u_guess(:,i)=(u_c+u_f)/2;
    else
        R=[Y2(j,4:6)' Y2(j,7:9)' Y2(j,10:12)'];
        m=Y2(j,16:18)';
        u_guess(:,i)=inv(4*MP.Kb2)*R'*m;
    end
        Rsd((i-1)*2+1:i*2)=u_guess(1:2,i)*curvFactor;
end
Rsd(2*FBG.num+1:2*FBG.num+4)=y0(19:22);

Rt_b=R0';
pt_b=-y0(1:3);
n1=length(t1);n2=length(t2);
for i=1:n1
    Y1(i,1:3)=(pt_b' + Y1(i,1:3))*Rt_b';
    Rtemp=Rt_b*[Y1(i,4:6);Y1(i,7:9);Y1(i,10:12)]';
    Y1(i,4:12)=[Rtemp(:,1)' Rtemp(:,2)' Rtemp(:,3)'];
end
for i=1:n2
    Y2(i,1:3)=(pt_b' + Y2(i,1:3))*Rt_b';
    Rtemp=Rt_b*[Y2(i,4:6);Y2(i,7:9);Y2(i,10:12)]';
    Y2(i,4:12)=[Rtemp(:,1)' Rtemp(:,2)' Rtemp(:,3)'];
end
Y1=flipud(Y1);Y2=flipud(Y2);
t1=flipud(t1);t2=flipud(t2);
p_output=zeros(21,1);
p_output(1:3)=Y1(11,1:3)';
p_output(4:6)=Y1(21,1:3)';
p_output(7:9)=Y1(31,1:3)';
p_output(10:12)=Y2(2,1:3)';
p_output(13:15)=Y2(12,1:3)';
p_output(16:18)=Y2(22,1:3)';
p_output(19:21)=Y2(28,1:3)';


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
% Date 20201031
% Ver e1.1
%%-----------------------------------------------------------------------%%