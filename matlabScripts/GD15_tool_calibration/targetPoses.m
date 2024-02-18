T0=[1 0 0 0;0 1 0 0;0 0 1 75; 0 0 0 1];

T1=[1 0 0 0;0 1 0 0;0 0 1 102.5;0 0 0 1];

R2=EulerToRotMat([0 0 -45]);p2=[34 34 136.5]';
T2=[R2 p2;0 0 0 1];
T2a=[R2*Expm([0 0 pi/2]') p2;0 0 0 1];
T2b=[R2*Expm([0 0 pi/4]') p2;0 0 0 1];

R3=EulerToRotMat([0 0 45]);p3=[-34 34 136.5]';
T3=[R3 p3;0 0 0 1];
T3a=[R3*Expm([0 0 pi/2]') p3;0 0 0 1];
T3b=[R3*Expm([0 0 pi/4]') p3;0 0 0 1];

R4=EulerToRotMat([45 0 -45]);p4=[-34 -34 68.5]';
T4=[R4 p4 ;0 0 0 1];
T4a=[R4*Expm([0 0 pi/2]') p4;0 0 0 1];
T4b=[R4*Expm([0 0 pi/4]') p4;0 0 0 1];

R5=EulerToRotMat([45 0 45]);p5=[34 -34 68.5]';
T5=[R5 p5;0 0 0 1];
T5a=[R5*Expm([0 0 pi/2]') p5;0 0 0 1];
T5b=[R5*Expm([0 0 pi/4]') p5;0 0 0 1];

Ra=EulerToRotMat([10 0 -45])*Expm([0 0 pi/6]');
Rb=EulerToRotMat([10 0 45])*Expm([0 0 -pi/6]');
Rc=EulerToRotMat([-10 0 -45])*Expm([0 0 pi/6]');
Rd=EulerToRotMat([-10 0 45])*Expm([0 0 -pi/6]');
Re=EulerToRotMat([-45 0 -45])*Expm([0 0 pi/6]');
Rf=EulerToRotMat([-45 0 45])*Expm([0 0 -pi/6]');
Rg=EulerToRotMat([45 0 -45])*Expm([0 0 pi/6]');
Rh=EulerToRotMat([45 0 45])*Expm([0 0 -pi/6]');
% p_offset=[-7.48 8.06 0]';
p_offset=[0 0 0]';
TA=[Ra [35 35 200]'+p_offset;...
    0 0 0 1];
TB=[Rb [-35 35 200]'+p_offset;...
    0 0 0 1];
TC=[Rc [-35 -35 200]'+p_offset;...
    0 0 0 1];
TD=[Rd [35 -35 200]'+p_offset;...
    0 0 0 1];
TE=[Re [60 60 80]'+p_offset;...
    0 0 0 1];
TF=[Rf [-60 60 80]'+p_offset;...
    0 0 0 1];
TG=[Rg [-60 -60 80]'+p_offset;...
    0 0 0 1];
TH=[Rh [60 -60 80]'+p_offset;...
    0 0 0 1];