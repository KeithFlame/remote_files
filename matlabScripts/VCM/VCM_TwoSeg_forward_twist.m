%clc;clear;
%% ===<<Bottom-to-Up Cosserat model, Two Segments>>=== %%
%%====================List of funs and vars====================%%
% (i)Functions:
% shootingOpt-shooting method nested in an optimization framework
% forShooting-guess-residue shooting shell for forward kinematics
% odeCosserat1_twist-(external) universal cosserat integration kernal
%model for unloaded shape.
% (ii)Variables:
% MP-mechanical parameter struct of the robot
% qa-actuation lengths (phi and d can be included)
% Fe, Me-external tip load
% fe, le-distributed load
%%-------------------------------------------------------------%%

%% ===Input Specifications=== %%
e1=[1 0 0]';e2=[0 1 0]';e3=[0 0 1]';
%---mechanical and structural parameters
MP.E=50e9;%Rod Young's modules
MP.mu=0.33;%Rod Poisson rate
MP.G=MP.E/2/(1+MP.mu);%shear modulus
MP.d1=0.50e-3;%Rod diameters
MP.d2=0.40e-3;
MP.d0=.1e-3;%center structure
MP.ds=.6e-3;%stem structure
MP.rho=0.85e-3;%Rod pitch circle radius
MP.L=0e-3;%Robot stem length
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
MP.Ke0=diag([MP.A0*MP.G MP.A0*MP.G MP.A0*MP.E]);MP.Kb0=diag([MP.E*MP.I0 MP.E*MP.I0 2*MP.G*MP.I0]);
MP.Kes=diag([MP.As*MP.G MP.As*MP.G MP.As*MP.E]);MP.Kbs=diag([MP.E*MP.Is MP.E*MP.Is 2*MP.G*MP.Is]);

%---inuput table
qa=[0 0 0 0 0 0]' + [ 0 0 ...
    0 ...
    0 ...
    0 ...
    0]'*pi*MP.rho;

Fe=[0 0 0]';Me=[0 0 0.005]';
fe=[0 0 0]';le=[0 0 0]';%distributed not active in this version

%% ====Execute=== %%
[Guess,t1,t2,y1,y2]=shootingOpt(qa,Fe,Me,fe,le,MP);
size1=size(y1(:,1:3));size2=size(y2(:,1:3));
n1=size1(1);n2=size2(1);
figure(1);cla;
[hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP,[0.08 0.17 0.55]);
[hAx]=plotAxis(0.002,eye(4));
[hAx]=plotAxis(0.002,[y1(n1,4:6)' y1(n1,7:9)' y1(n1,10:12)' y1(n1,1:3)'; 0 0 0 1]);
[hAx]=plotAxis(0.002,[y2(n2,4:6)' y2(n2,7:9)' y2(n2,10:12)' y2(n2,1:3)'; 0 0 0 1]);
axis equal;
%---plot curvatures---%
%u=calcCurvature(y1,y2,t1,t2,MP);

%% ===Model Functions=== %%
function [Guess,t1,t2,y1,y2]=shootingOpt(qa,Fe,Me,fe,le,MP)
dGuess=eye(12)*1e-5;
lambda=5e-10;
eps=1e-5;
plotFlag=0;

nc=[0 0 0]';mc=[0 0 0]';v_diff=[0 0 0 0]';v_comm=[0 0]';
Guess=[nc;mc;v_diff;v_comm];
[Rsd,t1,y1,t2,y2]=forShooting(Guess,qa,Fe,Me,fe,le,MP);

if(plotFlag == 1)
    size1=size(y1(:,1:3));size2=size(y2(:,1:3));
    n1=size1(1);n2=size2(1);
    figure(1);cla;
    [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP);
end

while(norm(Rsd)>eps)
    disp(['residue0 = ' num2str(norm(Rsd))]);

    for i=1:12
        [Rsd_]=forShooting(Guess+dGuess(:,i),qa,Fe,Me,fe,le,MP);
        J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
    end

    Guess=Guess-inv(J'*J+lambda*eye(12))*J'*(Rsd);
    [Rsd,t1,y1,t2,y2]=forShooting(Guess,qa,Fe,Me,fe,le,MP);
     
    if(plotFlag==1)
        pause(0.01);
        delete(hShape);
    	size1=size(y1(:,1:3));size2=size(y2(:,1:3));
        n1=size1(1);n2=size2(1);
        [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1,n2],MP);
    end
end


end

function [Rsd,t1,y1,t2,y2]=forShooting(Guess,qa,Fe,Me,fe,le,MP)

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
end



%% ========================Basic Info========================= %%
% Forward kinematics model for multi-backbone continuum robots.  
% By YuyangChen
% Date 20210126
% Ver ft1.0
%%-----------------------------------------------------------------------%%