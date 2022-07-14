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
e1=[1 0 0]';e2=[0 1 0]';e3=[0 0 1]';

MP.E=55e9;%Rod Young's modules
MP.mu=0.33;%Rod Poisson rate
MP.d1=0.50e-3;%Rod diameters
MP.d2=0.40e-3;
MP.rho=0.85e-3;%Rod pitch circle radius
MP.L=500e-3;%Robot stem length
MP.L1=100e-3;%Robot seg1 length
MP.L2=20e-3;%Robot seg2 length
MP.Lr=10e-3;%Robot rigid seg length
MP.Lg=15.0e-3;%Robot gipper length
MP.r11=[1 0 0]'*MP.rho;MP.r12=[0 1 0]'*MP.rho;
MP.r21=[1/sqrt(2) 1/sqrt(2) 0]'*MP.rho;MP.r22=[-1/sqrt(2) 1/sqrt(2) 0]'*MP.rho;
MP.Q1=[S(MP.r11)*e3 S(MP.r12)*e3 S(MP.r21)*e3 S(MP.r22)*e3];
MP.Q2=[S(MP.r21)*e3 S(MP.r22)*e3];
MP.I1=pi*MP.d1^4/64;MP.A1=pi*MP.d1^2/4;
MP.I2=pi*MP.d2^4/64;MP.A2=pi*MP.d2^2/4;
MP.G=MP.E/2/(1+MP.mu);MP.J=2*MP.I1;
MP.Ke1=diag([3e8 3e8 MP.A1*MP.E]);MP.Kb1=diag([MP.E*MP.I1 MP.E*MP.I1 10]);
MP.Ke2=diag([3e8 3e8 MP.A2*MP.E]);MP.Kb2=diag([MP.E*MP.I2 MP.E*MP.I2 10]);

% specify actuation length and external loads
qa=[0 0 ...
    0.0 ...
    -1.1*pi/4 ...
    -sqrt(2)*1.1*pi/4+0.8*pi ...
    -sqrt(2)*1.1*pi/4+0.8*pi ]'*MP.rho; % PHI D 
qa=zeros(6,1);
%inplace rot qa
  %qa=[0 0.03 -0.0020 0.0004 -0.0005 -0.0011]';
%qa=[0 10 -0.736431 0 -0.314704 0.314704]'*1e-3;
Fe=[0.05 -0. 0.]';Me=[0 0 0]';
fe=[0 0 0]';le=[0 0 0]';%distributed not active in this version

%% ====Execute=== %%
[Guess,t1,t2,y1,y2]=shootingOpt(qa,Fe,Me,fe,le,MP);
size1=size(y1(:,1:3));size2=size(y2(:,1:3));
n1=size1(1);n2=size2(1);
figure(1);%cla;
[hAx]=plotAxis(0.02,eye(4));
[hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP,[0.08 0.17 0.55]);
[hAx]=plotAxis(0.005,[y1(n1,4:6)' y1(n1,7:9)' y1(n1,10:12)' y1(n1,1:3)'; 0 0 0 1]);
[hAx]=plotAxis(0.005,[y2(n2,4:6)' y2(n2,7:9)' y2(n2,10:12)' y2(n2,1:3)'; 0 0 0 1]);

p=y2(end,1:3)';R=[y2(end,4:6);y2(end,7:9);y2(end,10:12)]';T=[R p;0 0 0 1];
[F_est,t1est,t2est,y1est,y2est]=shootingOptEst(qa(3:6),T,Guess,fe,le,MP);

%---plot curvatures---%
u=calcCurvature(y1,y2,t1,t2,MP);
u1=u(:,1);u2=u(:,end);
for i=1:72
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
% figure(2);
% subplot(2,2,1);hold on;grid on;
% plot(strain_upper11,'-r');plot(strain_lower11,'-k');
% subplot(2,2,2);hold on;grid on;
% plot(strain_upper12,'-r');plot(strain_lower12,'-k');
% subplot(2,2,3);hold on;grid on;
% plot(strain_upper21,'-r');plot(strain_lower21,'-k');
% subplot(2,2,4);hold on;grid on;
% plot(strain_upper22,'-r');plot(strain_lower22,'-k');
% 
% u_bar=[mean(u(:,3:4),2) mean(u(:,13:14),2) mean(u(:,23:24),2) mean(u(:,33:34),2) mean(u(:,43:44),2) mean(u(:,53:54),2) mean(u(:,63:64),2)];
% u_C_=[u_bar(1,:) u_bar(2,:)]';
% 
% figure(2);hold on;grid on;cla;
% plotCurvature(u);
% plotAxis(10,eye(4));
% axis equal;
% % ---calculate energy---%
% E=0;
% for i=1:n1
%     E=E+0.5*u(:,i)'*(8*MP.Kb1)*u(:,i)*1e-3;
% end
% E1=E;
% for i=1:n2
%     E=E+0.5*u(:,n1+i)'*(4*MP.Kb2)*u(:,n1+i)*1e-3;
% end
% E12=E;
% for j=1:4
% E=E+0.5*([0 0 Guess(6+j)]*(2*MP.Ke2)*[0 0 Guess(6+j)]')*(MP.L1/2+MP.L2/2+MP.L);
% end
% disp(['Energy: E1=' num2str(E1) ', E12=' num2str(E12) ', Eall=' num2str(E)]);
%% ===Model Functions=== %%
function [Guess,t1,t2,y1,y2]=shootingOpt(qa,Fe,Me,fe,le,MP)
dGuess=eye(10)*1e-5;
lambda=5e-10; % Jacobian damping
eps=1e-5; % residue tolerance
plotFlag=0;

nc=[0 0 0]';mc=[0 0 0]';v=[0 0 0 0]';Guess=[nc;mc;v]; % guessed base force moment strain
[Rsd,t1,y1,t2,y2]=forShooting(Guess,qa,Fe,Me,fe,le,MP);

if(plotFlag == 1)
    size1=size(y1(:,1:3));size2=size(y2(:,1:3));
    n1=size1(1);n2=size2(1);
    figure(1);cla;
    [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP);
end
tic
while(norm(Rsd)>eps)
    disp(['residue0 = ' num2str(norm(Rsd))]);

    for i=1:10 % finite differencing for Jacobian of initial guess
        [Rsd_]=forShooting(Guess+dGuess(:,i),qa,Fe,Me,fe,le,MP);
        J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
    end

    Guess=Guess-inv(J'*J+lambda*eye(10))*J'*(Rsd); % update guess
    [Rsd,t1,y1,t2,y2]=forShooting(Guess,qa,Fe,Me,fe,le,MP); % number IVP of equal to number of guessed values
     toc   
    if(plotFlag==1)
        pause(0.02);
        delete(hShape);
    	size1=size(y1(:,1:3));size2=size(y2(:,1:3));
        n1=size1(1);n2=size2(1);
        [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1,n2],MP);
    end
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
plotFlag=0;

% nc=[0 0 0]';
% nc=Fe0;
% mc=[0 0 0]';v=[0 0 0 0]';
%Guess=[nc(1:2);mc;v];
% Guess=[nc;mc;v];
N_ukn=length(Guess);dGuess=eye(N_ukn)*1e-5;

[Rsd,t1,y1,t2,y2]=estShooting(Guess,qa,Tg,fe,le,MP);

if(plotFlag == 1)
    size1=size(y1(:,1:3));size2=size(y2(:,1:3));
    n1=size1(1);n2=size2(1);
    figure(1);%cla;
    [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP);
end
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
    if k==10&&norm(Rsd)>eps
        disp(['Est no converge. residue0 = ' num2str(norm(Rsd))]);
        break
    end
end
%F_est = [Guess(1:2);0];
F_est=Guess(1:3);
end

function [Rsd,t1,y1,t2,y2]=estShooting(Guess,qa,Tg,fe,le,MP)
if(length(qa)==4)
    p0=zeros(3,1);R0=eye(3);
else
    p0=[0 0 qa(2)]';R0=Expm([0 0 qa(1)]');
end
pg=Tg(1:3,4);Rg=Tg(1:3,1:3);

%nc=[Guess(1:2);0];
%mc=Guess(3:5);
%v=zeros(3,4);v(end,:)=Guess(6:9)';
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
% Forward kinematics model for multi-backbone continuum robots.  
% By YuyangChen
% Date 20200524
% Ver f1.0
%%-----------------------------------------------------------------------%%