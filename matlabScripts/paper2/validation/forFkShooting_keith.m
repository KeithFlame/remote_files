function [Rsd,MBP,t0,y0,t1,y1,t2,y2,t3,y3]=forFkShooting_keith(Guess,qa,ksi,MBP)
% stiff
ksi1= ksi(1:4);
ksi2= ksi(5:8);
MBP1 = MBP(1);
MBP2 = MBP(2);
MBP1 = MBP1.refreshLso(qa(2)*1000);
MBP2 = MBP2.refreshLso(qa(8)*1000);
MBP1 = MBP1.resetK12(ksi(9:10));
MBP2 = MBP2.resetK12(ksi(11:12));
MBP = [MBP1 MBP2];

vec61=[-8.372 93.501 94.648 4.045 -128.746 122.638]';
T10 =fromX2T(vec61);T10(1:3,4)=T10(1:3,4)/1000;
vec62=[2.252 94.141 95.537 7.092 129.456 -120.715]';
T20 =fromX2T(vec62);T20(1:3,4)=T20(1:3,4)/1000;
dr11 = [-61.559 41.486 87.021];
dr11 = fromaxang2rotm(dr11);
dr12=[-36.468 -137.416 -3.163];
dr12 = fromaxang2rotm(dr12);
dr21=[49.250 -67.259 62.779];
dr21 = fromaxang2rotm(dr21);
dr22=[7.378 -1.811 35.310];
dr22 = fromaxang2rotm(dr22);

% arm1
nc1=Guess(1:3); % base internal force
mc1=Guess(4:6); % base internal load
v1=zeros(3,4);v1(end,:)=Guess(7:10)'; % rod elongation strain

T10(1:3,1:3)=T10(1:3,1:3)*eul2rotm([qa(1) 0 0]);
p10=T10(1:3,4);R10=T10(1:3,1:3);
y10_=[p10' R10(:,1)' R10(:,2)' R10(:,3)' nc1' mc1' 0 0 0 0]';


% Seg 0
if(MBP1.Ls)>0
    [t10,y10]=odeCosserat_keith(y10_,v1,0,ksi1,MBP1);
    yL10=y10(end,:)';
else
    t10 = 0;
    yL10 = y10_;
    y10 = y10_';
end

% Seg 1
[t11,y11]=odeCosserat_keith(yL10,v1,1,ksi1,MBP1); % first seg y=p,R,n,m,q for each row
R11=[y11(end,4:6);y11(end,7:9);y11(end,10:12)]'; % R1e
ynew=y11(end,:)';ynew(1:3)=ynew(1:3)+R11*[0 0 MBP1.Lr]'; % move from 1e to 2b
ynew(16:18)=ynew(16:18)+skewMatrix_keith(R11*[0 0 MBP1.Lr]')*ynew(13:15)...
    +2*(cross(R11*MBP1.r11,R11*MBP1.Ke1*v1(:,1))+...
    cross(R11*MBP1.r12,R11*MBP1.Ke1*v1(:,2)));
y11 = [y11;ynew'];
tnew=t11(end)+MBP1.Lr;
t11 = [t11;tnew];
yL11_ = ynew;
yL11=ynew([1:18 21 22]);

% Seg 2
[t12,y12]=odeCosserat_keith(yL11,v1,2,ksi1,MBP1); % second seg
R12=[y12(end,4:6);y12(end,7:9);y12(end,10:12)]'; % R2e
ynew=y12(end,:)';ynew(1:3)=ynew(1:3)+R12*[0 0 MBP1.Lg]'; % move from 2e to g
ynew(16:18)=ynew(16:18)+skewMatrix_keith(R12*[0 0 MBP1.Lg]')*ynew(13:15)...
    +2*(cross(R12*MBP1.r21,R12*MBP1.Ke2*v1(:,3))+...
    cross(R12*MBP1.r22,R12*MBP1.Ke2*v1(:,4)));
R12_ = R12*dr11;
ynew(4:12)=[R12_(:,1);R12_(:,2);R12_(:,3)];
y12 = [y12;ynew'];
tnew=t12(end)+MBP1.Lg;
t12 = [t12;tnew];
yL12_ = ynew;
yL12=ynew(1:19);yL12(19)=0;

% Seg 3
[t13,y13]=odeCosserat_keith(yL12,v1,3,ksi1,MBP1);
R13=[y13(end,4:6);y13(end,7:9);y13(end,10:12)]';
yL13=y13(end,:)';

qe11=(MBP1.Lstem+MBP1.L1)*[v1(3,1) v1(3,2)]'; % elongation seg1
qe12=(MBP1.Lstem+MBP1.L1+MBP1.L2+MBP1.Lr)*[v1(3,3) v1(3,4)]'; % elongation seg2


%% arm2
nc2=Guess(11:13); % base internal force
mc2=Guess(14:16); % base internal load
v2=zeros(3,4);v2(end,:)=Guess(17:20)'; % rod elongation strain

T20(1:3,1:3)=T20(1:3,1:3)*eul2rotm([qa(7) 0 0]);
p20=T20(1:3,4);R20=T20(1:3,1:3);
y20_=[p20' R20(:,1)' R20(:,2)' R20(:,3)' nc2' mc2' 0 0 0 0]';

% Seg 0
if(MBP2.Ls)>0
    [t20,y20]=odeCosserat_keith(y20_,v2,0,ksi2,MBP2);
    yL20=y20(end,:)';
else
    t20 = 0;
    yL20 = y20_;
    y20 = y20_';
end

% Seg 1
[t21,y21]=odeCosserat_keith(yL20,v2,1,ksi2,MBP2); % first seg y=p,R,n,m,q for each row
R21=[y21(end,4:6);y21(end,7:9);y21(end,10:12)]'; % R1e
ynew=y21(end,:)';ynew(1:3)=ynew(1:3)+R21*[0 0 MBP2.Lr]'; % move from 1e to 2b
ynew(16:18)=ynew(16:18)+skewMatrix_keith(R21*[0 0 MBP2.Lr]')*ynew(13:15)...
    +2*(cross(R21*MBP2.r11,R21*MBP2.Ke1*v2(:,1))+...
    cross(R21*MBP2.r12,R21*MBP2.Ke1*v2(:,2)));
y21 = [y21;ynew'];
tnew=t21(end)+MBP2.Lr;
t21 = [t21;tnew];
yL21_ = ynew;
yL21=ynew([1:18 21 22]);

% Seg 2
[t22,y22]=odeCosserat_keith(yL21,v2,2,ksi2,MBP2); % second seg
R22=[y22(end,4:6);y22(end,7:9);y22(end,10:12)]'; % R2e
ynew=y22(end,:)';ynew(1:3)=ynew(1:3)+R22*[0 0 MBP2.Lg]'; % move from 2e to g
ynew(16:18)=ynew(16:18)+skewMatrix_keith(R22*[0 0 MBP2.Lg]')*ynew(13:15)...
    +2*(cross(R22*MBP2.r21,R22*MBP2.Ke2*v2(:,3))+...
    cross(R22*MBP2.r22,R22*MBP2.Ke2*v2(:,4)));
R22_ = R22*dr21;
ynew(4:12)=[R22_(:,1);R22_(:,2);R22_(:,3)];
y22 = [y22;ynew'];
tnew=t22(end)+MBP2.Lg;
t22 = [t22;tnew];
yL22_ = ynew;
yL22=ynew(1:19);yL22(19)=0;

% Seg 3
[t23,y23]=odeCosserat_keith(yL22,v2,3,ksi2,MBP2);
R23=[y23(end,4:6);y23(end,7:9);y23(end,10:12)]';
yL23=y23(end,:)';

qe21=(MBP2.Lstem+MBP2.L1)*[v2(3,1) v2(3,2)]'; % elongation seg1
qe22=(MBP2.Lstem+MBP2.L1+MBP2.L2+MBP2.Lr)*[v2(3,3) v2(3,4)]'; % elongation seg2



t0 = [t20;0;t10];
y0 = [y20;zeros(1,22);y10];
t1 = [t21;0;t11];
y1 = [y21;zeros(1,22);y11];
t2 = [t22;0;t12];
y2 = [y22;zeros(1,20);y12];
t3 = [t23;0;t13];
y3 = [y23;zeros(1,19);y13];

Rsd=zeros(20,1);
Rsd(1:3)=yL13(13:15)+yL23(13:15)-[0 -0 0]'; % boundary conditions
Rsd(4:6)=yL13(16:18)+yL23(16:18);
Rsd(7:9)=yL13(1:3)-yL23(1:3);
% axang = rotm2axang(R23'*(R13*eul2rotm([0 pi 0])))';
% Rsd(10:12)=axang(1:3)*axang(4);
Rrsd1 = R13*dr12;
Rrsd2 = R23*dr22;
Rsd(10:12)=invSkewMatrix_keith(Rrsd1*Rrsd2'-Rrsd2*Rrsd1');

Rsd(13:14) =yL11_(19:20)-(qa(3:4)+qe11); % path length
Rsd(15:16)=yL12_(19:20)-(qa(5:6)+qe12);
Rsd(17:18) =yL21_(19:20)-(qa(9:10)+qe21); % path length
Rsd(19:20)=yL22_(19:20)-(qa(11:12)+qe22);

end