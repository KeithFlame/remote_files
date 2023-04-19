function [Rsd,t0,y0,t1,y1,t2,y2,t3,y3]=forShooting_keith(Guess,qa,ksi)
% do an IVP shooting
Fe = 0;Me = 0;
% arm1
nc1=Guess(1:3); % base internal force
mc1=Guess(4:6); % base internal load
v1=zeros(3,4);v1(end,:)=Guess(7:10)'; % rod elongation strain
ksi1= ksi(1:4);
Ls1=double(0);
Lr1 = double(10e-3);
Lg1 = double(15e-3);
L11 = double(0.1);
L21 = double(0.02);
Lstem1 = double(0.6);
% Idx = int8(0);
r1_12=[0.0000    0.0025         0]';
r1_11=[ 0.0025 0 0]';
r1_22=[0.0019    0.0019         0]';
r1_21=[0.0019   -0.0019         0]';
Ke1_1 = [    1.0659         0         0
         0    1.0659         0
         0         0    2.8353]*1e4;
Ke1_2 = [    0.7559         0         0
         0    0.7559         0
         0         0    2.0106]*1e4;
T10 = eye(4);
T10(1:3,4)=[-28.7300   66.8106  105.0004]'*1e-3;
T10(1:3,3)=-[-0.3667    0.9298    0.0326]';T10(1:3,3)=T10(1:3,3)/norm(T10(1:3,3));
T10(1:3,2)=[-0.2555   -0.1882    0.9483]';
T10(1:3,1)=cross(T10(1:3,2),T10(1:3,3));T10(1:3,1)=T10(1:3,1)/norm(T10(1:3,1));
T10(1:3,2)=cross(T10(1:3,3),T10(1:3,1));
T10(1:3,1:3)=T10(1:3,1:3)*eul2rotm([qa(1) 0 0]);
p10=T10(1:3,4);R10=T10(1:3,1:3);
% p10=[0 0 0]';R10=eul2rotm([qa(1) 0 0]);
y10=[p10' R10(:,1)' R10(:,2)' R10(:,3)' nc1' mc1' 0 0 0 0]';
Idx = int8(0);
if Ls1>0
    [t10,y10]=odeCosserat_keith(y10,v1,Idx,ksi1);
    y10_=y10(end,:);
else
    t10 = 0;
    y10 = y10';
    y10_=y10;
end
Idx = int8(1);
[t11,y11]=odeCosserat_keith(y10_,v1,Idx,ksi1); % first seg y=p,R,n,m,q for each row
R11=[y11(end,4:6);y11(end,7:9);y11(end,10:12)]'; % R1e
ynew=y11(end,:);ynew(1:3)=y11(end,1:3)+[0 0 1]*Lr1*R11'; % move from 1e to 2b
y11 = [y11;ynew];
tnew=t11(end)+Lr1;
t11 = [t11;tnew];
yL11=y11(end,:)';

y11_=[yL11(1:3);yL11(4:15);cross(R11*[0 0 Lr1]',yL11(13:15))+yL11(16:18)+2*(cross(R11*r1_11,R11*Ke1_1*v1(:,1))+...
    cross(R11*r1_12,R11*Ke1_1*v1(:,2)));yL11(21:22)]; % ? start of 2nd seg

Idx = int8(2);
[t12,y12]=odeCosserat_keith(y11_,v1,Idx,ksi1); % second seg
R12=[y12(end,4:6);y12(end,7:9);y12(end,10:12)]'; % R2e
ynew=y12(end,:);ynew(1:3)=y12(end,1:3)+[0 0 1]*Lg1*R12'; % move from 2e to g
y12 = [y12;ynew];
tnew=t12(end)+Lg1;
t12 = [t12;tnew];
yL12=y12(end,:)';

qe11=(Lstem1+L11)*[v1(3,1) v1(3,2)]'; % elongation seg1
qe12=(Lstem1+L11+L21+Lr1)*[v1(3,3) v1(3,4)]'; % elongation seg2

R12_ = R12 * eul2rotm([0 -pi/2 0]);
% R12_ = R12 * eul2rotm([-1.5850    0.8312    1.8071]);


y12_=[yL12(1:3);R12_(:,1); R12_(:,2); R12_(:,3);cross(R12*[0 0 Lg1]',yL12(13:15))+yL12(13:15);yL12(16:18)+2*(cross(R12*r1_21,R12*Ke1_2*v1(:,3))+...
    cross(R12*r1_22,R12*Ke1_2*v1(:,4)));0]; % ? start of 3nd seg

Idx = int8(3);
[t13,y13]=odeCosserat_keith(y12_,v1,Idx,ksi1); % second seg
R13=[y13(end,4:6);y13(end,7:9);y13(end,10:12)]'; % R2e
yL13=y13(end,:)';


% arm2
nc2=Guess(11:13); % base internal force
mc2=Guess(14:16); % base internal load
v2=zeros(3,4);v2(end,:)=Guess(17:20)'; % rod elongation strain
ksi2 = ksi(5:8);
Ls2 = double(0);
Lr2 = double(10e-3);
Lg2 = double(15e-3);
L12 = double(0.1);
L22 = double(0.02);
Lstem2 = double(0.6);

r2_12=[0.0000    0.0025         0]';
r2_11=[ 0.0025 0 0]';
r2_22=[0.0019    0.0019         0]';
r2_21=[0.0019   -0.0019         0]';
Ke2_1 = [    1.0659         0         0
         0    1.0659         0
         0         0    2.8353]*1e4;
Ke2_2 = [    0.7559         0         0
         0    0.7559         0
         0         0    2.0106]*1e4;

T20 = eye(4);
T20(1:3,4)=[-134.7150   48.7399  107.0360]'*1e-3;
T20(1:3,3)=-[-0.8081    0.5818    0.0922]';T20(1:3,3)=T20(1:3,3)/norm(T20(1:3,3));
T20(1:3,2)=[-0.1020   -0.3581    0.9281]';
T20(1:3,1)=cross(T20(1:3,2),T20(1:3,3));T20(1:3,1)=T20(1:3,1)/norm(T20(1:3,1));
T20(1:3,2)=cross(T20(1:3,3),T20(1:3,1));
T20(1:3,1:3)=T20(1:3,1:3)*eul2rotm([qa(7) 0 0]);
p20=T20(1:3,4);R20=T20(1:3,1:3);
% p20=[-0.02 0 0]';R20=eul2rotm([qa(7) 0 0]);
y20=[p20' R20(:,1)' R20(:,2)' R20(:,3)' nc2' mc2' 0 0 0 0]';
Idx = int8(0);
if Ls2>0
    [t20,y20]=odeCosserat_keith(y20,v2,Idx,ksi2);
    y20_=y20(end,:);
else
    t20 = 0;
    y20 = y20';
    y20_=y20;
end

Idx = int8(1);
[t21,y21]=odeCosserat_keith(y20_,v2,Idx,ksi2); % first seg y=p,R,n,m,q for each row
R21=[y21(end,4:6);y21(end,7:9);y21(end,10:12)]'; % R1e
ynew=y21(end,:);ynew(1:3)=y21(end,1:3)+[0 0 1]*Lr2*R21'; % move from 1e to 2b
y21 = [y21;ynew];
tnew=t21(end)+Lr2;
t21 = [t21;tnew];
yL21=y21(end,:)';

y21_=[yL21(1:3);yL21(4:15);cross(R21*[0 0 Lr2]',yL21(13:15))+yL21(16:18)+2*(cross(R21*r2_11,R21*Ke2_1*v2(:,1))+...
    cross(R21*r2_12,R21*Ke2_1*v2(:,2)));yL21(21:22)]; % ? start of 2nd seg

Idx = int8(2);
[t22,y22]=odeCosserat_keith(y21_,v2,Idx,ksi2); % second seg
R22=[y22(end,4:6);y22(end,7:9);y22(end,10:12)]'; % R2e
ynew=y22(end,:);ynew(1:3)=y22(end,1:3)+[0 0 1]*Lg2*R22'; % move from 2e to g
y22 = [y22;ynew];
tnew=t22(end)+Lg2;
t22 = [t22;tnew];
yL22=y22(end,:)';

qe21=(Lstem2+L12)*[v2(3,1) v2(3,2)]'; % elongation seg1
qe22=(Lstem2+L12+L22+Lr2)*[v2(3,3) v2(3,4)]'; % elongation seg2

R22_ = R22 * eul2rotm([0 pi/2 0]);
% R22_ = R22 * eul2rotm([1.9264   -0.4921    0.7374]);

y22_=[yL22(1:3);R22_(:,1); R22_(:,2); R22_(:,3);yL22(13:15);cross(R22*[0 0 Lg2]',yL22(13:15))+...
    yL22(16:18)+2*(cross(R22*r2_21,R22*Ke2_2*v2(:,3))+...
    cross(R22*r2_22,R22*Ke2_2*v2(:,4)));0]; % ? start of 2nd seg

Idx = int8(3);
[t23,y23]=odeCosserat_keith(y22_,v2,Idx,ksi2); % second seg
R23=[y23(end,4:6);y23(end,7:9);y23(end,10:12)]'; % R2e
yL23=y23(end,:)';



t0 = [t10;0;t20];
y0 = [y10;zeros(1,22);y20];
t1 = [t11;0;t21];
y1 = [y11;zeros(1,22);y21];
t2 = [t12;0;t22];
y2 = [y12;zeros(1,20);y22];
t3 = [t13;0;t23];
y3 = [y13;zeros(1,19);y23];



Rsd=zeros(20,1);
Rsd(1:3)=Fe-yL13(13:15)-yL23(13:15); % boundary conditions
Rsd(4:6)=Me-yL13(16:18)-yL23(16:18);
Rsd(7:9)=yL13(1:3)-yL23(1:3);
axang = rotm2axang(R23'*(R13*eul2rotm([0 pi 0])))';
Rsd(10:12)=axang(1:3)*axang(4);
% % % % % Rsd(1:3)=Fe-yL12(13:15)-yL22(13:15); % boundary conditions
% % % % % Rsd(4:6)=Me-yL12(16:18)-2*(cross(R12*MBP1.r21,R12*MBP1.Ke1*v1(:,3))+...
% % % % %     cross(R12*MBP1.r22,R12*MBP1.Ke1*v1(:,4)))...
% % % % %     -yL22(16:18)-2*(cross(R22*MBP2.r21,R22*MBP2.Ke1*v2(:,3))+...
% % % % %     cross(R22*MBP2.r22,R22*MBP2.Ke1*v2(:,4)));
% % % % % Rsd(7:9)=yL12(1:3)-yL22(1:3);
% % % % % axang = rotm2axang(R23'*(R13*eul2rotm([0 pi 0])))';
% % % % % Rsd(10:12)=axang(1:3)*axang(4)*0;

Rsd(13:14) =yL11(19:20)-(qa(3:4)+qe11); % path length
Rsd(15:16)=yL12(19:20)-(qa(5:6)+qe12);
Rsd(17:18) =yL21(19:20)-(qa(9:10)+qe21); % path length
Rsd(19:20)=yL22(19:20)-(qa(11:12)+qe22);
end