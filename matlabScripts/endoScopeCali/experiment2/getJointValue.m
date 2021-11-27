function f=getJointValue(endoPsi)


result_name = join([getenv('VSARMCALIBPATH'), '\conf\', 'opt_result.log']);
opt_result = load(result_name);
LS = opt_result(1:4);
gamma3 = -opt_result(end) / 100;
outfilename = join([getenv('VSARMCALIBPATH'), '\calib_pic_coord_out.log']);
M_raw = importdata(outfilename);
M = str_rep(M_raw);
P1st = M(1, 13:15);
A=load('autoCali_20210221/newCost/logRead/logRead/Config.log');
P1st=A(1,13:15);
arm_id_read = load('arm_id.log');
t=load(['map/', num2str(arm_id_read), '/t.log']);
P1st=P1st+[t/2 0 0];
% LS=[17.2241    8.3457   30.6355   15.8762];
% gamma3=-6.9327/100;
% P1st=[-4.59803, 8.70767, 104.899];

% flag=[1 1];
verify_cfg_idx = evalin('base', 'verify_cfg_idx');
flag_list=[
    1 1;
    1 -1;
    -1 -1;
    -1 1];
flag = flag_list(verify_cfg_idx, :);



L1=LS(1);Lr=LS(2);L2=LS(3);Lg=LS(4);
% scaler=0:0.1:1;
% PHI=endoPsi(1);
% Ls=endoPsi(2);
THETA1=endoPsi(3);
DELTA1=endoPsi(4);
THETA2=endoPsi(5);
DELTA2=endoPsi(6);
k2=THETA2/L2;
k1=THETA1/L1;

%Ls
T1=eye(4);
s1=[0 0;0 0;0 0];

%segment1 circular
if THETA1==0
    T2=[1 0 0 0
        0 1 0 0
        0 0 1 L1
        0 0 0 1];
%     s2=T1*[0*scaler
%         0*scaler
%         L1*scaler
%         ones(1,length(scaler))];s2=s2([1 2 3],:);%Segment1 straight
else
    cosTHETA1=cos(THETA1);sinTHETA1=sin(THETA1);cosDELTA1=cos(DELTA1);sinDELTA1=sin(DELTA1);
    T2=[(cosDELTA1)^2*(cosTHETA1-1)+1 sinDELTA1*cosDELTA1*(cosTHETA1-1) cosDELTA1*sinTHETA1 cosDELTA1*(1-cosTHETA1)/k1
        sinDELTA1*cosDELTA1*(cosTHETA1-1) (cosDELTA1)^2*(1-cosTHETA1)+cosTHETA1 sinDELTA1*sinTHETA1 sinDELTA1*(1-cosTHETA1)/k1
        -cosDELTA1*sinTHETA1 -sinDELTA1*sinTHETA1 cosTHETA1 sinTHETA1/k1
        0 0 0 1];
%     s2=T1*[cosDELTA1*(1-cos(scaler*THETA1))/k1;
%         sinDELTA1*(1-cos(scaler*THETA1))/k1;
%         sin(scaler*THETA1)/k1;
%         ones(1,length(scaler))];s2=s2([1 2 3],:);%Segment2 bent
end

%Lr
T3=[1 0 0 0
    0 1 0 0
    0 0 1 Lr
    0 0 0 1];
% s3=T1*T2*[0 0;0 0;0 Lr;1 1];s3=s3([1 2 3],:);

%segment2 circular
if THETA2==0
    T4=[1 0 0 0
        0 1 0 0
        0 0 1 L2
        0 0 0 1];
%     s4=T1*T2*T3*[0*scaler
%         0*scaler
%         L2*scaler
%         ones(1,length(scaler))];s4=s4([1 2 3],:);%Segment2 straight
else
    cosTHETA2=cos(THETA2);sinTHETA2=sin(THETA2);cosDELTA2=cos(DELTA2);sinDELTA2=sin(DELTA2);
    T4=[(cosDELTA2)^2*(cosTHETA2-1)+1 sinDELTA2*cosDELTA2*(cosTHETA2-1) cosDELTA2*sinTHETA2 cosDELTA2*(1-cosTHETA2)/k2
        sinDELTA2*cosDELTA2*(cosTHETA2-1) (cosDELTA2)^2*(1-cosTHETA2)+cosTHETA2 sinDELTA2*sinTHETA2 sinDELTA2*(1-cosTHETA2)/k2
        -cosDELTA2*sinTHETA2 -sinDELTA2*sinTHETA2 cosTHETA2 sinTHETA2/k2
        0 0 0 1];
%     s4=T1*T2*T3*[cosDELTA2*(1-cos(scaler*THETA2))/k2;
%         sinDELTA2*(1-cos(scaler*THETA2))/k2;
%         sin(scaler*THETA2)/k2;
%         ones(1,length(scaler))];s4=s4([1 2 3],:);%Segment2 bent
end

%end effector
% gamma3=0;
T5=[cos(gamma3) -sin(gamma3) 0 0
    sin(gamma3) cos(gamma3) 0 0
    0 0 1 Lg
    0 0 0 1];
P1st=reshape(P1st,[3 1]);
P1tc=[0 0 sum(LS)]'+T5(1:3,1:3)*P1st;
flag=sign(flag);
t=0.5;
dp=t*[960 540];
z=norm([960 540]);
dDire=[dp.*flag z]/norm([dp.*flag z]);
dDire=reshape(dDire,[3 1]);

% s5=T1*T2*T3*T4*[0 0;0 0;0 Lg;1 1];s5=s5([1 2 3],:);

T=T1*T2*T3*T4*T5;
Pend1tc=T(1:3,4);
dDire1tc=T(1:3,1:3)*dDire;
p=(P1tc-Pend1tc)/norm(P1tc-Pend1tc);
f=1-dot(dDire1tc,p);
end