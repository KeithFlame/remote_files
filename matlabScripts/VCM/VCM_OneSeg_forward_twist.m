function [u,pe]=VCM_OneSeg_forward_twist(qa,Fe,Me)
if(nargin == 1)
    Fe =[0 0 0]';
    Me = [0 0 0]';
end
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
%tic;
%% ===Input Specifications=== %%
e1=[1 0 0]';e2=[0 1 0]';e3=[0 0 1]';

MP.E=50e9;%Rod Young's modules
MP.mu=0.33;%Rod Poisson rate
MP.d1=0.37e-3;%Rod diameters
MP.d0=0.3e-3;%segment central rod
MP.ds=1e-3;%stem central rod
MP.rho=0.85e-3;%Rod pitch circle radius
MP.L1=37.5e-3;%Robot seg1 length
MP.Lf=MP.L1;
MP.L=0e-3;%Robot stem length
MP.Lr=0e-3;%Robot rigid seg length
MP.Lg=0e-3;
MP.r11=[1 0 0]'*MP.rho;MP.r12=[0 1 0]'*MP.rho;
MP.Q1=[S(MP.r11)*e3 S(MP.r12)*e3];
MP.I1=pi*MP.d1^4/64;MP.A1=pi*MP.d1^2/4;
MP.I0=pi*MP.d0^4/64;MP.A0=pi*MP.d0^2/4;
MP.Is=pi*MP.ds^4/64;MP.As=pi*MP.ds^2/4;
MP.G=MP.E/2/(1+MP.mu);
MP.J1=2*MP.I1;MP.J0=2*MP.I0;MP.Js=2*MP.Is;
MP.Ke1=diag([MP.A1*MP.G MP.A1*MP.G MP.A1*MP.E]);MP.Kb1=diag([MP.E*MP.I1 MP.E*MP.I1 MP.G*MP.J1]);
MP.Ke0=diag([MP.A0*MP.G MP.A0*MP.G MP.A0*MP.E]);MP.Kb0=diag([MP.E*MP.I0 MP.E*MP.I0 MP.G*MP.J0]);
MP.Kes=diag([MP.As*MP.G MP.As*MP.G MP.As*MP.E]);MP.Kbs=diag([MP.E*MP.Is MP.E*MP.Is MP.G*MP.Js]);
MP.Ke0=diag([1e1 1e1 1e4]);MP.Kb0=diag([1e-5 1e-5 1e-4]);
fe=[0 0 0]';le=[0 0 0]';%distributed not active in this version

%% ====Execute=== %%
[Guess,t1,y1]=shootingOpt(qa,Fe,Me,fe,le,MP);
%toc;
pe=y1(end,1:3)';
size1=size(y1(:,1:3));size2=1;t2=[t1(end)];y2=[y1(end,:)];
n1=size1(1);n2=size2(1);
% figure(1);%cla;
% [hAx]=plotAxis(0.02,eye(4));
% [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP,[0.08 0.17 0.55]);
% [hAx]=plotAxis(0.01,[y1(n1,4:6)' y1(n1,7:9)' y1(n1,10:12)' y1(n1,1:3)'; 0 0 0 1]);
% [hAx]=plotAxis(0.01,[y2(n2,4:6)' y2(n2,7:9)' y2(n2,10:12)' y2(n2,1:3)'; 0 0 0 1]);
% axis equal;

end
%% ===Model Functions=== %%
function [Guess,t1,y1]=shootingOpt(qa,Fe,Me,fe,le,MP)
dGuess=eye(9)*1e-5;
lambda=5e-10;
eps=1e-5;
plotFlag=0;
tic;
nc=[0 0 0]';mc=[0 0 0]';v=[0 0 0]';Guess=[nc;mc;v];
[Rsd,t1,y1]=forShooting(Guess,qa,Fe,Me,fe,le,MP);

if(plotFlag == 1)
    size1=size(y1(:,1:3));y2=y1(end,:);
    n1=size1(1);n2=1;
    figure(1);%cla;
    [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1 n2],MP);
end

while(norm(Rsd)>eps)
    disp(['residue0 = ' num2str(norm(Rsd))]);

    for i=1:9
        [Rsd_]=forShooting(Guess+dGuess(:,i),qa,Fe,Me,fe,le,MP);
        J(:,i)=(Rsd_-Rsd)/norm(dGuess(:,i));
    end

    Guess=Guess-inv(J'*J+lambda*eye(9))*J'*(Rsd);
    [Rsd,t1,y1]=forShooting(Guess,qa,Fe,Me,fe,le,MP);
        
    if(plotFlag==1)
        pause(0.02);
        delete(hShape);
    	size1=size(y1(:,1:3));size2=size(y2(:,1:3));
        n1=size1(1);n2=size2(1);
        [hShape]=plotShape([y1(:,1:3);y2(:,1:3)],[y1(:,4:12);y2(:,4:12)],[n1,n2],MP);
    end
    toc;
end
if(plotFlag==1)
    %axis([-0.01 0.03 -0.01 0.03 0 0.04]);
end
end

function [Rsd,tf,yf]=forShooting(Guess,qa,Fe,Me,fe,le,MP)
%modeling of the base stem
nc=Guess(1:3);
mc=Guess(4:6);
v=zeros(3,2);v(end,:)=Guess(7:8)';

%p0=inv(4*MP.Ke1+MP.Kes)*[0 0 nc(3)]'*MP.L;
%R0=expm(S(inv(4*MP.Kb1+MP.Kbs)*[0 0 mc(3)]'*MP.L));
p0=[0 0 0]';R0=eye(3);

y0=[p0' R0(:,1)' R0(:,2)' R0(:,3)' nc' mc' 0 0 0]';
[tf,yf]=odeCosserat_single_twist(y0,v,MP,fe,le,1e-3,[0 MP.Lf]);
tf(end+1)=tf(end);
yf(end+1,:)=yf(end,:);
yL1=yf(end,:)';
R1=[yL1(4:6) yL1(7:9) yL1(10:12)];
    qe1=(MP.L+MP.L1)*[v(3,1) v(3,2)]';
    Rsd=zeros(9,1);
    Rsd(1:3)=Fe-yL1(13:15);
    Rsd(4:6)=Me-yL1(16:18)-2*( cross(R1*MP.r11,R1*MP.Ke1*v(:,1))+cross(R1*MP.r12,R1*MP.Ke1*v(:,2)));
    Rsd(7:8) =yL1(19:20)-(qa(1:2)+qe1);  
    Rsd(9)=yL1(21)-Guess(9);
    
end

%% ========================Basic Info========================= %%
% Forward kinematics model for multi-backbone continuum robots.  
% By YuyangChen
% Date 20210118
% Ver tw1.0
%%-----------------------------------------------------------------------%%