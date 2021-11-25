clc;clear;cla
%% ===<<Base-to-tip Cosserat model, Two Segments>>=== %%
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
% MP.ell=0;
%random target generating
% pg=[(rand(2,1)-[0.5 0.5]')/15; rand*0.01+0.08];Rg=Expm(1.0*(rand(3,1)-0.5*ones(3,1)));
% Tg=[Rg pg;0 0 0 1];
%sequence target generating
% Rg=eye(3);

% yref=100*0.2e-3;zref=0.09;
pref1=[0;20e-3;0.07];
pref2=[20e-3;20e-3;0.07];
pref=[pref1 pref2];
phase=1;

Fe=[0 0 0]';Me=[0 0 0]'; Ke=50; % actual stiffness of the spring
fe=[0 0 0]';le=[0 0 0]';%distributed not active in this version
QA=zeros(6,200);
% Guess=zeros(5,1);Guess(2)=-0.001;
W_est=zeros(6,200);F_est=zeros(2,200);
g=zeros(200,12);
dqa=diag([1 1e2 1 1 1 1]*1e-4); % no shorter than the step length of the ode kernel!
dW=diag([1 1 1 1 1 1]*1e-3);
alpha=1; % force regulation gain
vlim=0.1; % linear velocity m/s
wlim=pi/2;
dt=0.01;
p_thres=1e-3; % m
F_thres=1; % N

%% ==== Main control loop === %%
% initial env interaction simulation
% actually forward kinematics
[Guess,t1,t2,y1,y2,U]=shootingOptSim(QA(:,1),Ke,Me,fe,le,MP,zeros(10,1));
p=y2(end,1:3)';R=[y2(end,4:6);y2(end,7:9);y2(end,10:12)]'; % measured end pose
T=[R p;0 0 0 1];
g(1,:)=y2(end,1:12);GuessEst=zeros(10,1);
size1=size(y1(:,1:3));size2=size(y2(:,1:3));
n1=size1(1);n2=size2(1);
figure(1);cla;
plotAxis(0.02,eye(4));
[hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP,[0.08 0.17 0.55]);
[hAx1]=plotAxis(0.005,[y1(n1,4:6)' y1(n1,7:9)' y1(n1,10:12)' y1(n1,1:3)'; 0 0 0 1]);
[hAx2]=plotAxis(0.005,[y2(n2,4:6)' y2(n2,7:9)' y2(n2,10:12)' y2(n2,1:3)'; 0 0 0 1]);

err_p=pref(:,phase)-p;err_r=Logm(eye(3)*R');
for j=1:100
%     xref=j*0.2e-3; % xref is forced
    
    % force estimation
    W=[Ke*([0;0;0.07]-p);0;0;0];
    [W_est(1:3,j),GuessEst,t1est,t2est,y1est,y2est]=shootingOptEst(QA(:,j),T,Guess,fe,le,MP); % not precise enough 1e-3
    F_est(1,j)=norm(W_est(1:3,j));
    GuessComp=[Guess GuessEst];
    
    % for constant curvature only
%     L1=QA(2,j)+MP.L1;L2=MP.L2;Lr=MP.Lr;Lg=MP.Lg;
%     theta1=norm(U(n1,1:2))*L1;delta1=atan2(U(n1,2),U(n1,1))-pi/2; % delta is right handed
%     theta2=norm(U(end,1:2))*L2;delta2=atan2(U(end,2),U(end,1))-pi/2;
%     T2=[y1(end-1,4:6)' y1(end-1,7:9)' y1(end-1,10:12)' y1(end-1,1:3)'; 0 0 0 1];
%     T3=T2\[y1(end,4:6)' y1(end,7:9)' y1(end,10:12)' y1(end,1:3)'; 0 0 0 1];
%     T4=T3\[y2(end-1,4:6)' y2(end-1,7:9)' y2(end-1,10:12)' y2(end-1,1:3)'; 0 0 0 1];
%     T=[y2(end,4:6)' y2(end,7:9)' y2(end,10:12)' y2(end,1:3)'; 0 0 0 1];
%     T5=T4\T;
%     J0=JacobianC3([0;0;theta1;L1;delta1;Lr;theta2;L2;delta2;Lg],eye(4),T2,T3,T4,T5,T);
    
    % compliance and Jacobian
    J=zeros(6,6);
    for i=1:6 % Jacobian(DoF_rsd,DoF_guess)
        if i==2
            pause(0.01);
        end
        [Guessj,t1j,t2j,y1j,y2j]=forShootingOpt(QA(:,j)+dqa(:,i),W_est(:,j),fe,le,MP);
        Rj=[y2j(end,4:6);y2j(end,7:9);y2j(end,10:12)]';
        J(:,i)=[(y2j(end,1:3)'-p);Logm(R*Rj')]/norm(dqa(:,i)); % !!!
    end
    C=zeros(6,6);% only w.r.t. force !!!
    for i=1:6 % Jacobian(DoF_rsd,DoF_guess)
        [Guessj,t1c,t2c,y1c,y2c]=forShootingOpt(QA(:,j),W_est(:,j)+dW(:,i),fe,le,MP);
        Rc=[y2c(end,4:6);y2c(end,7:9);y2c(end,10:12)]';
        C(:,i)=[(y2c(end,1:3)'-p);Logm(R*Rc')]/norm(dW(:,i)); % !!!
        if i==4
            pause(0.01);
        end
    end
    
    % environment stiffness
    if j==1
        KE=0;
    else
        KE=norm(W_est(:,j)-W_est(:,j-1))/norm(p-g(j-1,1:3)')
%         KE=0;
    end
    
    % desired end effector twist
    % sliding mode?
    if norm(err_p)<=1e-2
        v=vlim*norm(err_p)/1e-2;
    else
        v=vlim;
    end
    if norm(err_r)<=pi/3
        w=wlim*norm(err_r)/(pi/3);
    else
        w=wlim;
    end
    if norm(W_est(:,j))>F_thres
        % major task: force regulation, secondary task: position
        Jm=W_est(1:3,j)'*KE;Js=eye(3);
        pinvJm=pinv(Jm);
        xm_dot=alpha*pinvJm*(norm(W_est(:,j))^2-F_thres^2)/2;
        xs_dot=v*(pref(:,phase)-p)/norm(pref(:,phase)-p);
        x_dot=xm_dot+Js*(eye(3)-pinvJm*Jm)*(xs_dot-Js*xm_dot);
        x_dot=[x_dot;0;0;0];
    else
        if norm(err_r)<1e-4
            x_dot= [v*err_p/norm(err_p);0;0;0];
        else
%             x_dot= [v*err_p/norm(err_p);w*err_r/norm(err_r)];
            x_dot= [v*err_p/norm(err_p);0;0;0];
        end
    end
    
    % resolved rates actuation control
    qa_dot=pinv(J(:,2:6))*(eye(6)+C*KE)*x_dot; % weight?
%     qa_dot=pinv(J(:,2:6))*x_dot;
    QA(:,j+1)=QA(:,j)+[0;qa_dot]*dt;
    if QA(2,j+1)<0
        QA(2,j+1)=0;
    end
    
    % env interaction simulation
    % actually forward kinematics
    [Guess,t1,t2,y1,y2,U]=shootingOptSim(QA(:,j+1),Ke,Me,fe,le,MP,Guess);
    p=y2(end,1:3)';R=[y2(end,4:6);y2(end,7:9);y2(end,10:12)]'; % measured end pose
    T=[R p;0 0 0 1];
    g(j+1,:)=y2(end,1:12);
    GuessComp(:,1)=Guess;
    
    % plot
    size1=size(y1(:,1:3));size2=size(y2(:,1:3));
    n1=size1(1);n2=size2(1);
    figure(1);delete(hShape);delete(hAx1);delete(hAx2);
    [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP,[0.08 0.17 0.55]);
    [hAx1]=plotAxis(0.005,[y1(n1,4:6)' y1(n1,7:9)' y1(n1,10:12)' y1(n1,1:3)'; 0 0 0 1]);
    [hAx2]=plotAxis(0.005,[y2(n2,4:6)' y2(n2,7:9)' y2(n2,10:12)' y2(n2,1:3)'; 0 0 0 1]);
    line([p(1) 0],[p(2) 0],[p(3) 0.07]);
%     line([p(1) pref(1)],[p(2) pref(2)],[p(3) pref(3)]);
    view(3);
    pause(0.01);
    
    err_p=pref(:,phase)-p;
    err_r=Logm(eye(3)*R');
    if norm(err_p)<p_thres
        if phase==1
            phase=2;
        else
            break
        end
    end
end

function J=JacobianC3(para,T1,T2,T3,T4,T5,T)
% anticlockwise delta

p=T(1:3,4);R1b=T1([1 2 3],[1 2 3]);T2b=T1*T2*T3;R2b=T2b([1 2 3],[1 2 3]);
p2e=T4([1 2 3],4);T2bg=T4*T5;p2eg=T2bg([1 2 3],4)-p2e;
p1e=T2([1 2 3],4);T1bg=T2*T3*T4*T5;p1eg=T1bg([1 2 3],4)-p1e;
%PHI=q(1);
theta1=para(3);
L1=para(4);
delta1=para(5);
theta2=para(7);
L2=para(8);
delta2=para(9);
costheta1=cos(theta1);sintheta1=sin(theta1);
cosdelta1=cos(delta1);sindelta1=sin(delta1);
costheta2=cos(theta2);sintheta2=sin(theta2);
cosdelta2=cos(delta2);sindelta2=sin(delta2);

if abs(theta1)<1e-4
    J1v3=[cosdelta1*L1/2 0 0
          sindelta1*L1/2 0 0
          0              1 0];
    J1w3=[-sindelta1 0 0
          cosdelta1 0 0
          0         0 0];
else
    h1=(1-costheta1)/theta1;
    J1v3=[cosdelta1*L1*(sintheta1-h1)/theta1     cosdelta1*h1    -L1*sindelta1*h1
          sindelta1*L1*(sintheta1-h1)/theta1     sindelta1*h1     L1*cosdelta1*h1
          L1*(costheta1-sintheta1/theta1)/theta1 sintheta1/theta1 0];
    J1w3=[-sindelta1 0 -cosdelta1*sintheta1
           cosdelta1 0 -sindelta1*sintheta1
           0         0  1-costheta1];
end
if abs(theta2)<1e-4
    J2v2=[cosdelta2*L2/2 0
          sindelta2*L2/2 0
          0              0];
    J2w2=[-sindelta2 0
           cosdelta2 0
           0         0];
else
    h2=(1-costheta2)/theta2;
    J2v2=[cosdelta2*L2*(sintheta2-h2)/theta2    -L2*sindelta2*h2
          sindelta2*L2*(sintheta2-h2)/theta2     L2*cosdelta2*h2
          L2*(costheta2-sintheta2/theta2)/theta2 0];
    J2w2=[-sindelta2 -cosdelta2*sintheta2
           cosdelta2 -sindelta2*sintheta2
           0          1-costheta2];
end
Jv=[-S(p)*[0;0;1] R1b*(-S(p1eg)*J1w3+J1v3) R2b*(-S(p2eg)*J2w2+J2v2)];
Jw=[[0;0;1] R1b*J1w3 R2b*J2w2];
J=[Jv;Jw];
end

%% ===Model Functions=== %%
function [Guess,t1,t2,y1,y2,U]=shootingOptSim(qa,Ke,Me,fe,le,MP,Guess)
lambda=5e-10;
eps=1e-5;
plotFlag=0;
MP.ell=qa(2);

% nc=[0 0 0]';mc=[0 0 0]';v=[0 0 0 0]';Guess=[nc;mc;v]; % guessed base force moment strain
[Rsd,t1,y1,t2,y2,U]=forShootingSim(Guess,qa,Ke,Me,fe,le,MP);
N_ukn=length(Guess);dGuess=eye(N_ukn)*1e-5; % differential step length

if(plotFlag == 1)
    size1=size(y1(:,1:3));size2=size(y2(:,1:3));
    n1=size1(1);n2=size2(1);
    figure(1);%cla;
    [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP);
end
k=0;
while(norm(Rsd)>eps)
    k=k+1;
    %disp(['residue0 = ' num2str(norm(Rsd))]);
    
    J=zeros(length(Rsd),N_ukn);
    for i=1:N_ukn % Jacobian(DoF_rsd,DoF_guess)
        [Rsd_]=forShootingSim(Guess+dGuess(:,i),qa,Ke,Me,fe,le,MP);
        J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
    end

    Guess=Guess-(J'*J+lambda*eye(N_ukn))\J'*(Rsd); % LM
    [Rsd,t1,y1,t2,y2,U]=forShootingSim(Guess,qa,Ke,Me,fe,le,MP);
%     pause(0.02);    
    
    if(plotFlag==1)
        delete(hShape);
    	size1=size(y1(:,1:3));size2=size(y2(:,1:3));
        n1=size1(1);n2=size2(1);
        [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1,n2],MP);
    end
    if k==10&&norm(Rsd)>eps
        disp(['Sim no converge. residue0 = ' num2str(norm(Rsd))]);
        break
    end
end

end

function [Rsd,t1,y1,t2,y2,U]=forShootingSim(Guess,qa,Ke,Me,fe,le,MP)
% do an IVP shooting
% if(length(qa)==4)
    p0=zeros(3,1);R0=eye(3);
% else
%     p0=[0 0 qa(2)]';R0=Expm([0 0 qa(1)]');
% end

nc=Guess(1:3); % base internal force
mc=Guess(4:6); % base internal load
v=zeros(3,4);v(end,:)=Guess(7:10)'; % rod elongation strain

y0=[p0' R0(:,1)' R0(:,2)' R0(:,3)' nc' mc' 0 0 0 0]';
[~,y0,u0]=odeCosserat1(y0,v,0,MP,fe,le,1e-3);
y0_=y0(end,:);

[t1,y1,u1]=odeCosserat1(y0_,v,1,MP,fe,le,1e-3); % first seg y=p,R,n,m,q for each row
R1=[y1(end,4:6);y1(end,7:9);y1(end,10:12)]'; % R1e
y1(end+1,:)=y1(end,:);y1(end,1:3)=y1(end,1:3)+[0 0 1]*MP.Lr*R1'; % move from 1e to 2b
t1(end+1)=t1(end)+MP.Lr;
yL1=y1(end,:)';

y1_=[yL1(1:3);yL1(4:15);yL1(16:18)+2*(cross(R1*MP.r11,R1*MP.Ke1*v(:,1))+cross(R1*MP.r12,R1*MP.Ke1*v(:,2)));yL1(21:22)]; % start of 2nd seg

[t2,y2,u2]=odeCosserat1(y1_,v,2,MP,fe,le,1e-3); % second seg
R2=[y2(end,4:6);y2(end,7:9);y2(end,10:12)]'; % R2e
y2(end+1,:)=y2(end,:);y2(end,1:3)=y2(end,1:3)+[0 0 1]*MP.Lg*R2'; % move from 2e to g
t2(end+1)=t2(end)+MP.Lg;
yL2=y2(end,:)';

y1=[y0;y1];
qe1=(MP.L+MP.L1)*[v(3,1) v(3,2)]'; % elongation seg1
qe2=(MP.L+MP.L1+MP.L2+MP.Lr)*[v(3,3) v(3,4)]'; % elongation seg2
U=[u0;u1;u2];

Rsd=zeros(10,1);
 % boundary conditions
Rsd(1:3)=Ke*([0;0;MP.L1+MP.L2+MP.Lr+MP.Lg]-yL2(1:3))-yL2(13:15); % spring force
Rsd(4:6)=Me-yL2(16:18)-2*( cross(R2*MP.r21,R2*MP.Ke2*v(:,3))+cross(R2*MP.r22,R2*MP.Ke2*v(:,4)));
Rsd(7:8) =yL1(19:20)-(qa(3:4)+qe1); % path length
Rsd(9:10)=yL2(19:20)-(qa(5:6)+qe2);
end

function [F_est,Guess,t1,t2,y1,y2]=shootingOptEst(qa,Tg,Guess,fe,le,MP)
% Force estimation give qa and Tg
lambda=5e-9;
eps=1e-4;
plotFlag=0;
MP.ell=qa(2);

% nc=[0 0 0]';mc=[0 0 0]';v=[0 0 0 0]';
% %Guess=[nc(1:2);mc;v];
% Guess=[nc;mc;v];
N_ukn=length(Guess);dGuess=eye(N_ukn)*1e-5;

[Rsd,t1,y1,t2,y2]=estShootingFor(Guess,qa,Tg,fe,le,MP);

if(plotFlag == 1)
    size1=size(y1(:,1:3));size2=size(y2(:,1:3));
    n1=size1(1);n2=size2(1);
    figure(1);%cla;
    [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP);
end
k=0;
tic
while(norm(Rsd)>eps)
    k=k+1;
%     disp(['residue0 = ' num2str(norm(Rsd))]);
    
    J=zeros(length(Rsd),N_ukn);
    for i=1:N_ukn
        [Rsd_]=estShootingFor(Guess+dGuess(:,i),qa,Tg,fe,le,MP);
        J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
    end
    Guess=Guess-(J'*J+lambda*eye(N_ukn))\J'*(Rsd);
    [Rsd,t1,y1,t2,y2]=estShootingFor(Guess,qa,Tg,fe,le,MP);
    toc
%     if(norm(Rsd_-Rsd)/norm(Rsd)<1e-2)%steady
%         Rsd=Rsd_;
%         break;
%     else
%         Rsd=Rsd_;
%     end
    if(plotFlag==1)
        pause(0.02);
        delete(hShape);
    	size1=size(y1(:,1:3));size2=size(y2(:,1:3));
        n1=size1(1);n2=size2(1);
        [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1,n2],MP);
    end
    if k==20&&norm(Rsd)>eps
        disp(['Est no converge. residue0 = ' num2str(norm(Rsd))]);
        break
    end
end
%F_est = [Guess(1:2);0];
F_est=Guess(1:3);
end

function [Rsd,t1,y1,t2,y2]=estShootingFor(Guess,qa,Tg,fe,le,MP)
% if(length(qa)==4)
    p0=zeros(3,1);R0=eye(3);
% else
%     p0=[0 0 qa(2)]';R0=Expm([0 0 qa(1)]');
% end
pg=Tg(1:3,4);Rg=Tg(1:3,1:3);

%nc=[Guess(1:2);0];
%mc=Guess(3:5);
%v=zeros(3,4);v(end,:)=Guess(6:9)';
nc=Guess(1:3);
mc=Guess(4:6);
v=zeros(3,4);v(end,:)=Guess(7:10)';
step=1e-3;

y0=[p0' R0(:,1)' R0(:,2)' R0(:,3)' nc' mc' 0 0 0 0]';
[~,y0]=odeCosserat1(y0,v,0,MP,fe,le,1e-3);
y0_=y0(end,:);

[t1,y1]=odeCosserat1(y0_,v,1,MP,fe,le,step);
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

y1=[y0;y1];
qe1=(MP.L+MP.L1)*[v(3,1) v(3,2)]';
qe2=(MP.L+MP.L1+MP.L2+MP.Lr)*[v(3,3) v(3,4)]';

Rsd=zeros(10,1);
 % boundary conditions
Rsd(1:3)=yL2(1:3,:)-pg; % end pose
Rsd(4:6)=Logm(R2'*Rg);
Rsd(7:8) =yL1(19:20)-(qa(3:4)+qe1); % path length
Rsd(9:10)=yL2(19:20)-(qa(5:6)+qe2);
end

function [Rsd,t1,y1,t2,y2]=estShootingBack(Guess,qa,Tg,fe,le,MP)
% if(length(qa)==4)
    p0=zeros(3,1);R0=eye(3);
% else
%     p0=[0 0 qa(2)]';R0=Expm([0 0 qa(1)]');
% end
pg=Tg(1:3,4);Rg=Tg(1:3,1:3);

%nc=[Guess(1:2);0];
%mc=Guess(3:5);
%v=zeros(3,4);v(end,:)=Guess(6:9)';
nc=Guess(1:3);
mc=Guess(4:6);
v=zeros(3,4);v(end,:)=Guess(7:10)';
step=1e-3;

y0=[p0' R0(:,1)' R0(:,2)' R0(:,3)' nc' mc' 0 0 0 0]';
[~,y0]=odeCosserat1(y0,v,0,MP,fe,le,1e-3);
y0_=y0(end,:);

[t1,y1]=odeCosserat1(y0_,v,1,MP,fe,le,step);
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

y1=[y0;y1];
qe1=(MP.L+MP.L1)*[v(3,1) v(3,2)]';
qe2=(MP.L+MP.L1+MP.L2+MP.Lr)*[v(3,3) v(3,4)]';

Rsd=zeros(10,1);
 % boundary conditions
Rsd(1:3)=yL2(1:3,:)-pg; % end pose
Rsd(4:6)=Logm(R2'*Rg);
Rsd(7:8) =yL1(19:20)-(qa(3:4)+qe1); % path length
Rsd(9:10)=yL2(19:20)-(qa(5:6)+qe2);
end

function [Guess,t1,t2,y1,y2]=forShootingOpt(qa,We,fe,le,MP)
dGuess=eye(10)*1e-5;
lambda=5e-10; % Jacobian damping
eps=1e-5; % residue tolerance
plotFlag=0;
MP.ell=qa(2);
Fe=We(1:3);Me=We(4:6);

nc=[0 0 0]';mc=[0 0 0]';v=[0 0 0 0]';Guess=[nc;mc;v]; % guessed base force moment strain
[Rsd,t1,y1,t2,y2]=forShooting(Guess,qa,Fe,Me,fe,le,MP);

if(plotFlag == 1)
    size1=size(y1(:,1:3));size2=size(y2(:,1:3));
    n1=size1(1);n2=size2(1);
    figure(1);cla;
    [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP);
end
% tic
k=0;
while(norm(Rsd)>eps)
    k=k+1;
%     disp(['residue0 = ' num2str(norm(Rsd))]);
    
    J=zeros(length(Rsd),10);
    for i=1:10 % finite differencing for Jacobian of initial guess
        [Rsd_]=forShooting(Guess+dGuess(:,i),qa,Fe,Me,fe,le,MP);
        J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
    end

    Guess=Guess-(J'*J+lambda*eye(10))\J'*(Rsd); % update guess
    [Rsd,t1,y1,t2,y2]=forShooting(Guess,qa,Fe,Me,fe,le,MP); % number IVP of equal to number of guessed values
%      toc   
    if(plotFlag==1)
        pause(0.02);
        delete(hShape);
    	size1=size(y1(:,1:3));size2=size(y2(:,1:3));
        n1=size1(1);n2=size2(1);
        [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1,n2],MP);
    end
    if k==10&&norm(Rsd)>eps
        disp(['Forward no converge. residue0 = ' num2str(norm(Rsd))]);
        break
    end
end


end

function [Rsd,t1,y1,t2,y2]=forShooting(Guess,qa,Fe,Me,fe,le,MP)
% do an IVP shooting

p0=zeros(3,1);R0=Expm([0 0 qa(1)]');

nc=Guess(1:3); % base internal force
mc=Guess(4:6); % base internal load
v=zeros(3,4);v(end,:)=Guess(7:10)'; % rod elongation strain

y0=[p0' R0(:,1)' R0(:,2)' R0(:,3)' nc' mc' 0 0 0 0]';
[~,y0]=odeCosserat1(y0,v,0,MP,fe,le,1e-3);
y0_=y0(end,:);

[t1,y1]=odeCosserat1(y0_,v,1,MP,fe,le,1e-3); % first seg y=p,R,n,m,q(path) for each row
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

y1=[y0;y1];
qe1=(MP.L+MP.L1)*[v(3,1) v(3,2)]'; % elongation seg1
qe2=(MP.L+MP.L1+MP.L2+MP.Lr)*[v(3,3) v(3,4)]'; % elongation seg2

Rsd=zeros(10,1);
Rsd(1:3)=Fe-yL2(13:15); % boundary conditions
Rsd(4:6)=Me-yL2(16:18)-2*( cross(R2*MP.r21,R2*MP.Ke2*v(:,3))+cross(R2*MP.r22,R2*MP.Ke2*v(:,4)));
Rsd(7:8) =yL1(19:20)-(qa(3:4)+qe1); % path length
Rsd(9:10)=yL2(19:20)-(qa(5:6)+qe2);
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