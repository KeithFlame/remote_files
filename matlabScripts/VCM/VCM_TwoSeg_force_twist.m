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



%% ===Input Specifications=== %%
e1=[1 0 0]';e2=[0 1 0]';e3=[0 0 1]';
%---mechanical and structural parameters
MP.E=50e9;%Rod Young's modules
MP.mu=0.33;%Rod Poisson rate
MP.G=MP.E/2/(1+MP.mu);%shear modulus
MP.d1=0.50e-3;%Rod diameters
MP.d2=0.40e-3;
MP.d0=1e-3;%center structure
MP.ds=2.5e-3;%stem structure
MP.rho=0.85e-3;%Rod pitch circle radius
MP.L=500e-3;%Robot stem length
MP.L1=37.5e-3;%Robot seg1 length
MP.L2=25e-3;%Robot seg2 length
MP.Lr=2.5e-3;%Robot rigid seg length
MP.Lg=5.0e-3;%Robot gipper length
MP.r11=[1 0 0]'*MP.rho;MP.r12=[0 1 0]'*MP.rho;
MP.r21=[1/sqrt(2) 1/sqrt(2) 0]'*MP.rho;MP.r22=[-1/sqrt(2) 1/sqrt(2) 0]'*MP.rho;
MP.Q1=[S(MP.r11)*e3 S(MP.r12)*e3 S(MP.r21)*e3 S(MP.r22)*e3];
MP.Q2=[S(MP.r21)*e3 S(MP.r22)*e3];
MP.I1=pi*MP.d1^4/64;MP.A1=pi*MP.d1^2/4;
MP.I2=pi*MP.d2^4/64;MP.A2=pi*MP.d2^2/4;
MP.I0=pi*MP.d0^4/64;MP.A0=pi*MP.d0^2/4;
MP.Is=pi*MP.ds^4/64;MP.As=pi*MP.ds^2/4;
MP.Ke1=diag([MP.A1*MP.G MP.A1*MP.G MP.A1*MP.E]);MP.Kb1=diag([MP.E*MP.I1 MP.E*MP.I1 2*MP.G*MP.I1]);
MP.Ke2=diag([MP.A2*MP.G MP.A2*MP.G MP.A2*MP.E]);MP.Kb2=diag([MP.E*MP.I2 MP.E*MP.I2 2*MP.G*MP.I2]);
MP.Ke0=diag([MP.A0*MP.G MP.A0*MP.G MP.A0*MP.E]);MP.Kb0=diag([1e-7 1e-7 1e-3]);
MP.Kes=diag([MP.As*MP.G MP.As*MP.G MP.As*MP.E]);MP.Kbs=diag([MP.E*MP.Is MP.E*MP.Is 2*MP.G*MP.Is]);
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
load('./1210/config3.mat');
u=config3.u;
u=Expm([0 0 -5/180*pi]')*u;
qa=config3.qa;
strain=config3.strain;
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

P_uz = eye(7)*1e-3;
Q_prc_uz = eye(7)*1e-3;

Me=zeros(3,1);fe=zeros(3,1);le=zeros(3,1);

delta_twist= zeros(7,1);
global Posi_1 Ori_1;
Ori_1=eye(3);Posi_1=[0 0 0]';
[p_obs,J_twist]=fbg_ShapePositions(u,delta_twist);

%figure(2);cla;axis on;
uz_fix=zeros(68,1);
for i=1:30
    u_obs = [u(1:2,1);u(1:2,2);u(1:2,3);u(1:2,4);u(1:2,5);u(1:2,6);u(1:2,7)]+randn(14,1)*0.1;
    %---time update
    Fe_ = Fe;
    P_ = P + Q_prc;
    %---measurement update
    uz_fix=zeros(68,1);
    [p_guess,t1,t2,y1,y2,C]=shootingOpt_fix_twist(qa,Fe_,Me,fe,le,MP,FBG,uz_fix);
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
        %figure(2);hold on;
        %plot(i,F_est(1),'*r');
        %plot(i,F_est(2),'*g');
        %plot(i,F_est(3),'*b');
        
        
    %=== torsion-based estimation
    %time update
    delta_twist_=delta_twist;
    P_uz_=P_uz+Q_prc_uz;
    %measurement update
    [p_obs2,t1,t2,y1,y2,uz_fix,C3]=shootingOpt_twist(qa,F_est,Me,fe,le,MP);
    [p_guess2,J_twist]=fbg_ShapePositions(u,delta_twist_);
        
    K2 = P_uz_*J_twist'*inv( J_twist*P_uz_*J_twist'+Q_obs );
    delta_twist = delta_twist+K2*(p_obs2 - p_guess2);
    P_uz = (eye(7)-K2*J_twist)*P_uz_;
        %---plot & disp
        size1=size(y1(:,1:3));size2=size(y2(:,1:3));
        n1=size1(1);n2=size2(1);
        figure(1);
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
        
    %re-draw shape
    [p_guess3,J_twist]=fbg_ShapePositions(u,delta_twist);P3=P_;Q3=Q_obs*100;
    
end


%%%%%=====fbg twist model: change to base rotation!


end


%% ===Model Functions=== %%
function [p_guess,t1,t2,y1,y2,Cp]=shootingOpt_fix_twist(qa,Fe,Me,fe,le,MP,FBG,uz_fix)
%num=9;
num=4;
dGuess=eye(num)*1e-5;
dF=eye(3)*1e-5;
lambda=5e-9;
eps=1e-5;
plotFlag=0;

v=[0 0 0 0]';
Guess=[v];

[Rsd,t1,y1,t2,y2,pO]=estShooting_fix_twist(Guess,qa,Fe,Me,fe,le,MP,FBG,uz_fix);


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
        [Rsd_,t1,y1,t2,y2,pO_]=estShooting_fix_twist(Guess+dGuess(:,i),qa,Fe,Me,fe,le,MP,FBG,uz_fix);
        J(:,i)=(Rsd_(15:18)-Rsd(15:18))/norm(dGuess(:,i));
    end
    %compliance matrix of positions
    for i=1:3
        [Rsd_,t1,y1,t2,y2,pO_]=estShooting_fix_twist(Guess,qa,Fe+dF(:,i),Me,fe,le,MP,FBG,uz_fix);
        C(:,i)=(Rsd_(1:14)-Rsd(1:14))/norm(dF(:,i));
        Cp(:,i)=(pO_-pO)/norm(dF(:,i));
    end
    %----
    if(norm(Rsd(15:18))<eps)
        break;
    end
    Guess=Guess-inv(J'*J+lambda*eye(num))*J'*(Rsd(15:18));
    [Rsd_,t1,y1,t2,y2,pO_]=estShooting_fix_twist(Guess,qa,[Fe(1:2);0],Me,fe,le,MP,FBG,uz_fix);
    
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

function [Rsd,t1,Y1,t2,Y2,p_output]=estShooting_fix_twist(Guess,qa,Fe,Me,fe,le,MP,FBG,uz_fix)
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
uz_fix2 = flipud(uz_fix(end-27:end));
uz_fix1 = flipud(uz_fix(2:40));
y2=[p2' R2(:,1)' R2(:,2)' R2(:,3)' nc' mc' q2']';
y2_=y2;y2_(1:3)=y2(1:3)-R2*[0 0 1]'*MP.Lg;
[t2,Y2]=odeCosserat_inverse_fix_twist(y2_,v,2,MP,fe,le,step,uz_fix2);
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

[t1,Y1]=odeCosserat_inverse_fix_twist(y1_,v,1,MP,fe,le,step,uz_fix1);
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

Rt_b=(R0*Expm([0 0 -1]*uz_fix(1)*(MP.L-qa(2))))';
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

%% ===Model Functions - Twisting === %%
function [p_guess,t1,t2,y1,y2,uz_fix,Cp]=shootingOpt_twist(qa,Fe,Me,fe,le,MP)
dGuess=eye(12)*1e-5;
lambda=5e-10;
eps=1e-5;
plotFlag=0;
dF=eye(3)*1e-5;
nc=[0 0 0]';mc=[0 0 0]';v_diff=[0 0 0 0]';v_comm=[0 0]';
Guess=[nc;mc;v_diff;v_comm];
[Rsd,t1,y1,t2,y2,pO]=forShooting_twist(Guess,qa,Fe,Me,fe,le,MP);

if(plotFlag == 1)
    size1=size(y1(:,1:3));size2=size(y2(:,1:3));
    n1=size1(1);n2=size2(1);
    figure(1);cla;
    [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP);
end

while(norm(Rsd)>eps)
    %disp(['residue0 = ' num2str(norm(Rsd))]);

    for i=1:12
        [Rsd_]=forShooting_twist(Guess+dGuess(:,i),qa,Fe,Me,fe,le,MP);
        J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
    end

    for i=1:3
        [Rsd_,t1,y1,t2,y2,pO_]=forShooting_twist(Guess,qa,Fe+dF(:,i),Me,fe,le,MP);
        Cp(:,i)=(pO_-pO)/norm(dF(:,i));
    end
    
    Guess=Guess-inv(J'*J+lambda*eye(12))*J'*(Rsd);
    [Rsd,t1,y1,t2,y2]=forShooting_twist(Guess,qa,Fe,Me,fe,le,MP);
    

    
    if(plotFlag==1)
        pause(0.01);
        delete(hShape);
    	size1=size(y1(:,1:3));size2=size(y2(:,1:3));
        n1=size1(1);n2=size2(1);
        [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1,n2],MP);
    end
end
u=calcCurvature(y1,y2,t1,t2,MP);
uz_fix=[u(3,1) u(3,4:42) u(3,42) u(3,44:end)]';
p_guess=zeros(21,1);
p_guess(1:3)=y1(13,1:3)';
p_guess(4:6)=y1(23,1:3)';
p_guess(7:9)=y1(33,1:3)';
p_guess(10:12)=y2(1,1:3)';
p_guess(13:15)=y2(11,1:3)';
p_guess(16:18)=y2(21,1:3)';
p_guess(19:21)=y2(27,1:3)';


end

function [Rsd,t1,y1,t2,y2,p_output]=forShooting_twist(Guess,qa,Fe,Me,fe,le,MP)

MP.ell=qa(2);
nc=Guess(1:3);
mc=Guess(4:6);
v=zeros(3,4);v(end,:)=Guess(7:10)';
q_diff=[0 0 0 0]';q_comm=0;

%base extension/insertion and base torsion/rotation
Extension_base=inv(4*MP.Ke2+4*MP.Ke1+MP.Kes)*[0 0 nc(3)]'*(MP.L-qa(2));
Torsion_base=inv(4*MP.Kb2+4*MP.Kb1+MP.Kbs)*[0 0 mc(3)]'*(MP.L-qa(2));
p0=Extension_base;
R0=Expm( Torsion_base+[0 0 qa(1)]' );

ys=[p0' R0(:,1)' R0(:,2)' R0(:,3)' nc' mc' q_comm q_diff']';
[ts,ys]=odeCosserat1_twist(ys,v,0,MP,fe,le,1e-3);
ys=[0 0 0 1 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;ys];
y0=ys(end,:)';

[t1,y1]=odeCosserat1_twist(y0,v,1,MP,fe,le,1e-3);
R1=[y1(end,4:6);y1(end,7:9);y1(end,10:12)]';
y1(end+1,:)=y1(end,:);y1(end,1:3)=y1(end,1:3)+[0 0 1]*MP.Lr*R1';
t1(end+1)=t1(end)+MP.Lr;
y1=[ys;y1];t1=[ts;t1];
yL1=y1(end,:)';

y1_=[yL1(1:3);yL1(4:15);yL1(16:18)+2*(cross(R1*MP.r11,R1*MP.Ke1*v(:,1))+cross(R1*MP.r12,R1*MP.Ke1*v(:,2)));yL1(19);yL1(22:23)];

[t2,y2]=odeCosserat1_twist(y1_,v,2,MP,fe,le,1e-3);
R2=[y2(end,4:6);y2(end,7:9);y2(end,10:12)]';
y2(end+1,:)=y2(end,:);y2(end,1:3)=y2(end,1:3)+[0 0 1]*MP.Lg*R2';
t2(end+1)=t2(end)+MP.Lg;
yL2=y2(end,:)';

qe1_diff=(MP.L+MP.L1)*[v(3,1) v(3,2)]';
qe2_diff=(MP.L+MP.L1+MP.L2+MP.Lr)*[v(3,3) v(3,4)]';
qe1_comm=(MP.L+MP.L1)*Guess(11);
qe2_comm=(MP.L+MP.L1+MP.L2+MP.Lr)*Guess(12);

Rsd=zeros(12,1);
Rsd(1:3)=Fe-yL2(13:15);
Rsd(4:6)=Me-yL2(16:18)-2*( cross(R2*MP.r21,R2*MP.Ke2*v(:,3))+cross(R2*MP.r22,R2*MP.Ke2*v(:,4)));
Rsd(7:8) =yL1(20:21)-(qa(3:4)+qe1_diff);%diff 1
Rsd(9:10)=yL2(20:21)-(qa(5:6)+qe2_diff);%diff 2
Rsd(11:12)=[yL1(19);yL2(19)]+sqrt((MP.L-qa(2))^2+(Torsion_base(3)*MP.rho)^2)-(MP.L-qa(2))+Extension_base(3)...
    -[qe1_comm;qe2_comm];%comm

p_output=zeros(21,1);
p_output(1:3)=y1(13,1:3)';
p_output(4:6)=y1(23,1:3)';
p_output(7:9)=y1(33,1:3)';
p_output(10:12)=y2(1,1:3)';
p_output(13:15)=y2(11,1:3)';
p_output(16:18)=y2(21,1:3)';
p_output(19:21)=y2(27,1:3)';
end

function [p_obs,J_twist]=fbg_ShapePositions(u,delta_twist)
p=[0 0 0]';R=eye(3);
p_obs=zeros(21,1);
p_rel=zeros(3,7);

R_rel=zeros(3,3,7);
R_acc=zeros(3,3,7);

J_v=zeros(3,7);
J_w=zeros(3,7);
figure(1);global Ori_1 Posi_1;
Ori_1=eye(3);Posi_1=[0 0 0]';
for i=1:7
    delta_p(i)=pi/2-atan2(u(2,i),u(1,i));
    theta_p(i)=10e-3*norm(u(:,i));
    L=10e-3;
    if(theta_p(i)~=0)
        p_rel(:,i)=L/theta_p(i)*[cos(delta_p(i)+delta_twist(i))*(1-cos(theta_p(i))) -sin(delta_p(i)+delta_twist(i))*(1-cos(theta_p(i))) sin(theta_p(i))]';
        R_rel(:,:,i)=Expm(-(delta_p(i)+delta_twist(i))*[0 0 1]')*Expm(theta_p(i)*[0 1 0]')*Expm(delta_p(i)*[0 0 1]');
        J_v(:,i)=L/theta_p(i)*(cos(theta_p(i))-1)*[sin(delta_p(i)+delta_twist(i)) cos(delta_p(i)+delta_twist(i)) 0]';
    else
        p_rel(:,i)=[0 0 L]';
        R_rel(:,:,i)=Expm([0 0 -delta_twist(i)]');
        J_v(:,i)=[0 0 0]';
    end
    J_w(:,i)=[0 0 -1]';
    p=p+R*p_rel(:,i);
    R=R*R_rel(:,:,i);
    p_obs(3*i-2:3*i) = p;
    R_acc(:,:,i)=R;
    
    Ori_1=Ori_1*Expm(delta_twist(i)*[0 0 -1]');
    PlotSnake(delta_p(i),theta_p(i),10e-3,[5e-4 1e3]);
    
end
J_twist=zeros(21,7);
for i=1:7%column  
    for j=1:i%row
        if(i~=1)
            J_twist(i*3-2:i*3,j)=...
                -R_acc(:,:,j)*S(R_acc(:,:,j)'*(p_obs(3*i-2:3*i)-p_obs(3*j-2:3*j)))*J_w(:,j)+J_v(:,j);
        else
            J_twist(1:3,1)=-S(p_obs(3*i-2:3*i)-p_obs(3*j-2:3*j))*J_w(:,1)+J_v(:,1);
        end
    end
end


end

function [u]=calcCurvature(y1,y2,t1,t2,MP)
size1=size(y1(:,1:3));size2=size(y2(:,1:3));
n1=size1(1);n2=size2(1);
u=zeros(3,n1+n2);
u(:,1)=inv(4*MP.Kb1+4*MP.Kb2+MP.Kbs)*[y1(1,4:6);y1(1,7:9);y1(1,10:12)]*y1(2,16:18)';

y_all=[y1(:,1:18);y2(:,1:18)];
for i =2:n1
    R=[y_all(i,4:6)' y_all(i,7:9)' y_all(i,10:12)'];
    n=y_all(i,13:15)';
    m=y_all(i,16:18)';
    u(:,i)=inv(4*MP.Kb1+4*MP.Kb2+MP.Kb0)*R'*m;
    %u_n(i)=norm(u(:,i));
    %u(3,i)=t1(i);
end
for i =n1+1:n1+n2
    R=[y_all(i,4:6)' y_all(i,7:9)' y_all(i,10:12)'];
    n=y_all(i,13:15)';
    m=y_all(i,16:18)';
    u(:,i)=inv(4*MP.Kb2+MP.Kb0)*R'*m;
    %u_n(i)=norm(u(:,i));
    %u(3,i)=t2(i-n1);
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