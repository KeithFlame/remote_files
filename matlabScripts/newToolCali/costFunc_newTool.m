function f=costFunc_newTool(x)
x1= x;
% x1(1)=x(1)*10;
% x1(2:3)=x(2:3)*100;
% x1(4:5)=x1(4:5)*10;
% x1(10)=x1(10)*10;
x=x1;
Tb = setTarget;
psi_b = setPsi;
qb = setActuation;
block_size = size(Tb,3);
To = zeros(4,4,5);
position_error = zeros(block_size,1);
angle_error = zeros(block_size,1);
SL = [x(3) x(4) x(5) 15]';
% dt=8;
for i = 1:block_size
    psi=calcPsiFromQ(qb(:,i),psi_b(:,i),x);
%     t=psi(2);
%     psi(2)=psi(1)-x(4)-x(5);
%     psi(1)=t;
    kmp=forward_tem_621(psi,SL,0.15,8,x(1));
    To(:,:,i)=[kmp.RO_g kmp.PO_g;[0 0 0 1]];
%     To(:,:,i) = FKcc_2segs_bending_keith(psi,SL);
%     To(3,4,i)=To(3,4,i)-dt;
    position_error(i)=norm(To(1:3,4,i)-Tb(1:3,4,i));
    axang=rotm2axang(To(1:3,1:3,i)'*Tb(1:3,1:3,i));
    angle_error(i)=abs(axang(4))*180/pi;
end

f=mean(position_error)+mean(angle_error);


end

function [config]=calcPsiFromQ(qa,psi_in,x)
%---------constant curvature model for unloaded robot--------------------%
% calculating actuation lengths from curvatures, for Liuxu's arm
%----Info
% input config = [l phi theta1s delta1 theta2 delta2], 
% only takes length l from config and l is {2e} from trocar in mm
% By Yuyang Chen
% Date 20230620
% Ver c1.0
%-------------------------------------------------------------------------%
%---constants
e1=[1 0 0]';e2=[0 1 0]';e3=[0 0 1]';
MP.E=50e9;%Rod Young's modules
MP.mu=0.25;%Rod Poisson rate
MP.d1=0.95e-3;%Rod diameter seg1
MP.d2=0.4e-3;%Rod diameter seg2
MP.d3=0.95e-3;%Rod diameter prox actuation
MP.rho1=2.5e-3;%Rod pitch circle radius seg1
MP.rho2=2.7e-3;%Rod pithc circle radius seg2
MP.rhop=x(10)*1e-3;%prox pitch circle radius of seg2
MP.rho3=8e-3;%prox pitch circle radius of seg3
MP.alpha = MP.rhop/MP.rho2;
MP.L1=x(3)*1e-3;%Robot seg1 length
MP.L2=x(4)*1e-3;%Robot seg2 length
MP.L3=20e-3;%Robot prox length
MP.Lc=64.5e-3;%robot cannula length
MP.La=x(2)*1e-3;%Robot stem length
MP.Lr=x(4)*1e-3;%Robot rigid seg length
MP.Lg=15e-3;%Robot gipper length
MP.r11=[1 0 0]'*MP.rho1;MP.r12=[0 1 0]'*MP.rho1;MP.r13=[-1 0 0]'*MP.rho1;MP.r14=[0 -1 0]'*MP.rho1;
MP.r21=[cos(21/180*pi) sin(21/180*pi) 0]'*MP.rho2;
MP.r22=[cos(37/180*pi) sin(37/180*pi) 0]'*MP.rho2;
MP.r23=[cos(53/180*pi) sin(53/180*pi) 0]'*MP.rho2;
MP.r24=[cos(69/180*pi) sin(69/180*pi) 0]'*MP.rho2;
MP.r25=[cos(111/180*pi) sin(111/180*pi) 0]'*MP.rho2;
MP.r26=[cos(127/180*pi) sin(127/180*pi) 0]'*MP.rho2;
MP.r27=[cos(143/180*pi) sin(143/180*pi) 0]'*MP.rho2;
MP.r28=[cos(159/180*pi) sin(159/180*pi) 0]'*MP.rho2;
MP.r29=[cos(201/180*pi) sin(201/180*pi) 0]'*MP.rho2;
MP.r2a=[cos(217/180*pi) sin(217/180*pi) 0]'*MP.rho2;
MP.r2b=[cos(233/180*pi) sin(233/180*pi) 0]'*MP.rho2;
MP.r2c=[cos(249/180*pi) sin(249/180*pi) 0]'*MP.rho2;
MP.r2d=[cos(291/180*pi) sin(291/180*pi) 0]'*MP.rho2;
MP.r2e=[cos(307/180*pi) sin(307/180*pi) 0]'*MP.rho2;
MP.r2f=[cos(323/180*pi) sin(323/180*pi) 0]'*MP.rho2;
MP.r2g=[cos(339/180*pi) sin(339/180*pi) 0]'*MP.rho2;
MP.r31=[1 0 0]'*MP.rho3;MP.r32=[0 1 0]'*MP.rho3;MP.r33=[-1 0 0]'*MP.rho3;MP.r34=[0 -1 0]'*MP.rho3;
MP.Q1=[S(MP.r11)*e3 S(MP.r12)*e3];
MP.Q2=[S(MP.r21)*e3 S(MP.r22)*e3 S(MP.r23)*e3 S(MP.r24)*e3 ...
    S(MP.r25)*e3 S(MP.r26)*e3 S(MP.r27)*e3 S(MP.r28)*e3];
MP.Q3=[S(MP.r31)*e3 S(MP.r32)*e3];
MP.I1=pi*MP.d1^4/64;MP.A1=pi*MP.d1^2/4;
MP.I2=pi*MP.d2^4/64;MP.A2=pi*MP.d2^2/4;
MP.I3=pi*MP.d3^4/64;MP.A3=pi*MP.d3^2/4;
MP.G=MP.E/2/(1+MP.mu);
MP.J1=2*MP.I1;MP.J2=2*MP.I2;MP.J3=2*MP.I3;
MP.Ke1=diag([MP.G*MP.A1 MP.G*MP.A1 MP.A1*MP.E]);
MP.Ke2=diag([MP.G*MP.A2 MP.G*MP.A2 MP.A2*MP.E]);
MP.Ke3=diag([MP.G*MP.A3 MP.G*MP.A3 MP.A3*MP.E]);
MP.Kb1=diag([MP.E*MP.I1 MP.E*MP.I1 2*MP.G*MP.I1]);
MP.Kb2=diag([MP.E*MP.I2 MP.E*MP.I2 2*MP.G*MP.I2]);
MP.Kb3=diag([MP.E*MP.I3 MP.E*MP.I3 2*MP.G*MP.I3]);
MP.zeta =0.15;
MP.L1x = 0;
MP.d = 8e-3;

l=psi_in(1)/1000;
config=psi_in;
if(l-MP.L2-MP.Lr+MP.d+MP.L1x>MP.L1)
    l1 = MP.L1;
    ls = l-MP.L2-MP.Lr-MP.L1+MP.d+MP.L1x;
else
    l1 = l-MP.L2-MP.Lr+MP.d+MP.L1x;
    ls = 0;
end

K1=diag([x(6) x(7) 1]);
K2=diag([x(8) x(9) 1]);


MatP = zeros(12,9);
MatL=zeros(12,12);
MatA=zeros(9,12);
MatK=zeros(9,9);

MatP(1:2,1:3)=(l1+ls*MP.zeta)*MP.Q1';
MatP(3:10,1:3)=(l1+ls*MP.zeta)*MP.Q2';
MatP(3:10,4:6)=MP.L2*MP.Q2';
MatP(3:10,7:9)=-MP.alpha*MP.L3*MP.Q2';
MatP(11:12,7:9)=-MP.L3*MP.Q3';

MatL(1:2,1:2)=diag([1 1])*(MP.L1+MP.La);
MatL(3:10,3:10)=diag([1 1 1 1 1 1 1 1])*(MP.L2+MP.Lr+MP.L1+MP.La+MP.Lc+MP.L3);
MatL(11:12,11:12)=diag([1 1])*MP.L3;
    
MatA(1:3,1:2)=MP.A1*MP.E*MP.Q1;
MatA(4:6,3:10)=MP.A2*MP.E*MP.Q2;
MatA(7:9,11:12)=MP.A3*MP.E*MP.Q3;

MatK(1:3,1:3)=4*MP.Kb1+16*MP.Kb2 + K1*MP.Kb1;
MatK(1:3,4:6)=-16*MP.Kb2 - K2*MP.Kb1;
MatK(4:6,4:6)=16*MP.Kb2 + K2*MP.Kb1;
MatK(7:9,4:6)=MP.alpha*(16*MP.Kb2 + K2*MP.Kb1);
MatK(7:9,7:9)=4*MP.Kb3+16*MP.Kb2;

if(length(qa)==12)
    qa_extend = qa/1000;
else% 4-dof qa
    qa_extend = [qa(1) qa(2) zeros(1,8) qa(3) qa(4)]'/1000;
end

    u=pinv(MatP+MatL*pinv(MatA)*MatK)*qa_extend;

u1=u(1:3);
u2=u(4:6);
u3=u(7:9);

%using NEW delta
config(3)=norm(u1)*(ls*MP.zeta+l1);
config(4)=-pi/2+atan2(u1(2),u1(1));
config(5)=MP.L2*norm(u2);
config(6)=-pi/2+atan2(u2(2),u2(1));
% config = limitPsiNum(config);
end

function [qa,MP]=calcQFromPsi(psi,x)
%---------constant curvature model for unloaded robot--------------------%
% calculating actuation lengths from curvatures
% ----Info
% By Yuyang Chen
% Date 20230620
% Ver c1.0
% input config = [l phi theta1s delta1 theta2 delta2],
% delta=atan2(y,x) and l is {2e} from trocar in mm
% unit meter inside
%-------------------------------------------------------------------------%
%---constants
e1=[1 0 0]';e2=[0 1 0]';e3=[0 0 1]';
MP.E=50e9;%Rod Young's modules
MP.mu=0.25;%Rod Poisson rate
MP.d1=0.95e-3;%Rod diameter seg1
MP.d2=0.4e-3;%Rod diameter seg2
MP.d3=0.95e-3;%Rod diameter prox actuation
MP.rho1=2.5e-3;%Rod pitch circle radius seg1
MP.rho2=2.7e-3;%Rod pithc circle radius seg2
MP.rhop=x(10)*1e-3;%prox pitch circle radius of seg2
MP.rho3=8e-3;%prox pitch circle radius of seg3
MP.alpha = MP.rhop/MP.rho2;
MP.L1=x(3)*1e-3;%Robot seg1 length
MP.L2=x(4)*1e-3;%Robot seg2 length
MP.L3=20e-3;%Robot prox length
MP.Lc=64.5e-3;%robot cannula length
MP.La=x(2)*1e-3;%Robot stem length
MP.Lr=x(4)*1e-3;%Robot rigid seg length
MP.Lg=15e-3;%Robot gipper length
MP.r11=[1 0 0]'*MP.rho1;MP.r12=[0 1 0]'*MP.rho1;MP.r13=[-1 0 0]'*MP.rho1;MP.r14=[0 -1 0]'*MP.rho1;
MP.r21=[cos(21/180*pi) sin(21/180*pi) 0]'*MP.rho2;
MP.r22=[cos(37/180*pi) sin(37/180*pi) 0]'*MP.rho2;
MP.r23=[cos(53/180*pi) sin(53/180*pi) 0]'*MP.rho2;
MP.r24=[cos(69/180*pi) sin(69/180*pi) 0]'*MP.rho2;
MP.r25=[cos(111/180*pi) sin(111/180*pi) 0]'*MP.rho2;
MP.r26=[cos(127/180*pi) sin(127/180*pi) 0]'*MP.rho2;
MP.r27=[cos(143/180*pi) sin(143/180*pi) 0]'*MP.rho2;
MP.r28=[cos(159/180*pi) sin(159/180*pi) 0]'*MP.rho2;
MP.r29=[cos(201/180*pi) sin(201/180*pi) 0]'*MP.rho2;
MP.r2a=[cos(217/180*pi) sin(217/180*pi) 0]'*MP.rho2;
MP.r2b=[cos(233/180*pi) sin(233/180*pi) 0]'*MP.rho2;
MP.r2c=[cos(249/180*pi) sin(249/180*pi) 0]'*MP.rho2;
MP.r2d=[cos(291/180*pi) sin(291/180*pi) 0]'*MP.rho2;
MP.r2e=[cos(307/180*pi) sin(307/180*pi) 0]'*MP.rho2;
MP.r2f=[cos(323/180*pi) sin(323/180*pi) 0]'*MP.rho2;
MP.r2g=[cos(339/180*pi) sin(339/180*pi) 0]'*MP.rho2;
MP.r31=[1 0 0]'*MP.rho3;MP.r32=[0 1 0]'*MP.rho3;MP.r33=[-1 0 0]'*MP.rho3;MP.r34=[0 -1 0]'*MP.rho3;
MP.Q1=[S(MP.r11)*e3 S(MP.r12)*e3];
MP.Q2=[S(MP.r21)*e3 S(MP.r22)*e3 S(MP.r23)*e3 S(MP.r24)*e3 ...
    S(MP.r25)*e3 S(MP.r26)*e3 S(MP.r27)*e3 S(MP.r28)*e3];
MP.Q3=[S(MP.r31)*e3 S(MP.r32)*e3];
MP.I1=pi*MP.d1^4/64;MP.A1=pi*MP.d1^2/4;
MP.I2=pi*MP.d2^4/64;MP.A2=pi*MP.d2^2/4;
MP.I3=pi*MP.d3^4/64;MP.A3=pi*MP.d3^2/4;
MP.G=MP.E/2/(1+MP.mu);
MP.J1=2*MP.I1;MP.J2=2*MP.I2;MP.J3=2*MP.I3;
MP.Ke1=diag([MP.G*MP.A1 MP.G*MP.A1 MP.A1*MP.E]);
MP.Ke2=diag([MP.G*MP.A2 MP.G*MP.A2 MP.A2*MP.E]);
MP.Ke3=diag([MP.G*MP.A3 MP.G*MP.A3 MP.A3*MP.E]);
MP.Kb1=diag([MP.E*MP.I1 MP.E*MP.I1 2*MP.G*MP.I1]);
MP.Kb2=diag([MP.E*MP.I2 MP.E*MP.I2 2*MP.G*MP.I2]);
MP.Kb3=diag([MP.E*MP.I3 MP.E*MP.I3 2*MP.G*MP.I3]);
MP.zeta =0.15;
MP.L1x = 0;
MP.d = 8e-3;

    
l=psi(1)/1000;
if(l-MP.L2-MP.Lr+MP.d+MP.L1x>MP.L1)
    l1 = MP.L1;
    ls = l-MP.L2-MP.Lr-MP.L1+MP.d+MP.L1x;
else
    l1 = l-MP.L2-MP.Lr+MP.d+MP.L1x;
    ls = 0;
end
theta1 = psi(3)*l1/(ls*MP.zeta+l1);
thetas = psi(3)*ls*MP.zeta/(ls*MP.zeta+l1);
delta1 = psi(4);
theta2 = psi(5);
delta2 = psi(6);
u1 = theta1/l1*[cos(delta1+pi/2) sin(delta1+pi/2) 0]';
u2 = theta2/MP.L2*[cos(delta2+pi/2) sin(delta2+pi/2) 0]';


K1=diag([x(6) x(7) 1]);
K2=diag([x(8) x(9) 1]);

MatP = zeros(12,9);
MatL=zeros(12,12);
MatA=zeros(9,12);
MatK=zeros(9,9);

MatP(1:2,1:3)=(l1+ls*MP.zeta)*MP.Q1';
MatP(3:10,1:3)=(l1+ls*MP.zeta)*MP.Q2';
MatP(3:10,4:6)=MP.L2*MP.Q2';
MatP(3:10,7:9)=-MP.alpha*MP.L3*MP.Q2';
MatP(11:12,7:9)=-MP.L3*MP.Q3';

MatL(1:2,1:2)=diag([1 1])*(MP.L1+MP.La);
MatL(3:10,3:10)=diag([1 1 1 1 1 1 1 1])*(MP.L2+MP.Lr+MP.L1+MP.La+MP.Lc+MP.L3);
MatL(11:12,11:12)=diag([1 1])*MP.L3;
    
MatA(1:3,1:2)=MP.A1*MP.E*MP.Q1;
MatA(4:6,3:10)=MP.A2*MP.E*MP.Q2;
MatA(7:9,11:12)=MP.A3*MP.E*MP.Q3;

MatK(1:3,1:3)=4*MP.Kb1+16*MP.Kb2 + K1*MP.Kb1;
MatK(1:3,4:6)=-16*MP.Kb2 - K2*MP.Kb1;
MatK(4:6,4:6)=16*MP.Kb2 + K2*MP.Kb1;
MatK(7:9,4:6)=MP.alpha*(16*MP.Kb2 + K2*MP.Kb1);
MatK(7:9,7:9)=4*MP.Kb3+16*MP.Kb2;

u3=pinv(MP.Q2')/MP.alpha/MP.L3*( MP.Q2'*(l1+MP.zeta*ls)*u1 ...
                               +(MP.Q2'*MP.L2+ (MP.L2+MP.Lr+MP.L1+MP.La+MP.Lc+MP.L3)/MP.A2/MP.E*pinv(MP.Q2)*(16*MP.Kb2+K2*MP.Kb1))*u2 );
                   
u=[u1;u2;u3];

qa=(MatP+MatL*pinv(MatA)*MatK)*u*1000;

end

function T = S( p )
%this function gives the skew-symmetric matrix of the vector p

if(length(p)==3)
    T=[0 -p(3) p(2); p(3) 0 -p(1);-p(2) p(1) 0];
elseif(length(p)==6)
    R=[0 -p(6) p(5); p(6) 0 -p(4);-p(5) p(4) 0];
    T=[R p(1:3);zeros(1,4)];
end
end

