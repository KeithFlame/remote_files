%% ===<<Bottom-to-Up Cosserat model, Two Segments>>=== %%
%%====================List of funs and vars====================%%
% (i)Functions:
% shootingOpt-shooting method nested in an optimization framework
% forShooting-guess-residue shooting shell for forward kinematics
% odeCosserat1-(external) universal cosserat integration kernal
% plotShape, plotAxis, drawCircle-plot funs
% (ii)Variables:
% MP-mechanical parameter struct of the robot
% qa-actuation lengths (phi and d can be included)
% Fe, Me-external tip load
% fe, le-distributed load
%%-------------------------------------------------------------%%

%% ===Input Specifications=== %%
function []=forwardKinematics(qa,MP)
if(nargin == 1)
e1=[1 0 0]';e2=[0 1 0]';e3=[0 0 1]';

MP.E=80e9;%Rod Young's modules
MP.mu=0.33;%Rod Poisson rate
MP.d1=0.50e-3;%Rod diameters
MP.d2=0.40e-3;
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
MP.G=MP.E/2/(1+MP.mu);MP.J=2*MP.I1;
MP.Ke1=diag([3e8 3e8 MP.A1*MP.E]);MP.Kb1=diag([MP.E*MP.I1 MP.E*MP.I1 10]);
MP.Ke2=diag([3e8 3e8 MP.A2*MP.E]);MP.Kb2=diag([MP.E*MP.I2 MP.E*MP.I2 10]);
MP.ell=0;
end
% qa=[0 ...
%     -1.1*pi/2 ...
%     -1.1*sqrt(2)*pi/4 + 1.3*pi ...
%     -1.1*sqrt(2)*pi/4 - 0*pi     ]'*MP.rho;
% qa=[1.0464 0.0082 -0.0013 0.0003 0.0014 -0.0003]';
% qa=[1.0485 0.0087 -0.0013 0.0003 0.0014 -0.0003]';
Fe=[0 0 0]';Me=[0 0 0]';
fe=[0 0 0]';le=[0 0 0]';%distributed not active in this version

%% ====Execute=== %%
[Guess,t1,t2,y1,y2]=shootingOpt(qa,Fe,Me,fe,le,MP);
size1=size(y1(:,1:3));size2=size(y2(:,1:3));
n1=size1(1);n2=size2(1);
figure(1);cla;
[hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP);
[hAx]=plotAxis(0.005,[y1(1,4:6)' y1(1,7:9)' y1(1,10:12)' y1(1,1:3)'; 0 0 0 1]);
[hAx]=plotAxis(0.005,[y1(n1,4:6)' y1(n1,7:9)' y1(n1,10:12)' y1(n1,1:3)'; 0 0 0 1]);
[hAx]=plotAxis(0.005,[y2(n2,4:6)' y2(n2,7:9)' y2(n2,10:12)' y2(n2,1:3)'; 0 0 0 1]);

end
%% ===Model Functions=== %%
function [Guess,t1,t2,y1,y2]=shootingOpt(qa,Fe,Me,fe,le,MP)
dGuess=eye(10)*1e-5;
lambda=5e-10;
eps=1e-5;
plotFlag=0;

nc=[0 0 0]';mc=[0 0 0]';v=[0 0 0 0]';Guess=[nc;mc;v];
[Rsd,t1,y1,t2,y2]=forShooting(Guess,qa,Fe,Me,fe,le,MP);

if(plotFlag == 1)
    size1=size(y1(:,1:3));size2=size(y2(:,1:3));
    n1=size1(1);n2=size2(1);
    figure(1);%cla;
    [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP);
end
while(norm(Rsd)>eps)
    %disp(['residue0 = ' num2str(norm(Rsd))]);
    for i=1:10
        [Rsd_]=forShooting(Guess+dGuess(:,i),qa,Fe,Me,fe,le,MP);
        J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
    end

    Guess=Guess-inv(J'*J+lambda*eye(10))*J'*(Rsd);
    [Rsd,t1,y1,t2,y2]=forShooting(Guess,qa,Fe,Me,fe,le,MP);
           
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
if(length(qa)==4)
    p0=zeros(3,1);R0=eye(3);
else
    p0=[0 0 qa(2)]';R0=Expm([0 0 qa(1)]');
end

nc=Guess(1:3);
mc=Guess(4:6);
v=zeros(3,4);v(end,:)=Guess(7:10)';

y0=[p0' R0(:,1)' R0(:,2)' R0(:,3)' nc' mc' 0 0 0 0]';
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
qe2=(MP.L+MP.L1+MP.L2+MP.Lr)*[v(3,3) v(3,4)]';

Rsd=zeros(10,1);
Rsd(1:3)=Fe-yL2(13:15);
Rsd(4:6)=Me-yL2(16:18)-2*( cross(R2*MP.r21,R2*MP.Ke2*v(:,3))+cross(R2*MP.r22,R2*MP.Ke2*v(:,4)));
Rsd(7:8) =yL1(19:20)-(qa(3:4)+qe1);
Rsd(9:10)=yL2(19:20)-(qa(5:6)+qe2);
end

%% ===Other Functions=== %%
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

%% ========================Basic Info========================= %%
% Forward kinematics model for multi-backbone continuum robots. This is the
% function form of the script BottomUpCosserat_TowSeg_for.m
% By YuyangChen
% Date 20200524
% Ver f1.0
%%-----------------------------------------------------------------------%%