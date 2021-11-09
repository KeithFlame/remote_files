% Ver. 1.8
% Author：Keith
% K1是变量，d是变量，L1x是变量,L2是定值 加入了L1和ζ
% Date： 09.21.2021
clear;
clc;
global Tst2tc;
global TLm2marker;
global isSimplified;
global Tst2tip_ini;
global Tst2tip_ini2;
global endoPsi_ini;
global path;
global XH1_7;

global OptSerial;
global optTimes;
global residualVal;
global gamma1;
residualVal=zeros(3000,1);
optTimes=0;

% path= 5919;%6136;5919 610018
path=load('../conf/configurationData/filePath.log');
fpath=['../conf/configurationData/arm',num2str(path),'/initPara.log'];
beforeOpt=load(fpath);
isSimplified=beforeOpt(end);
% [Tst2tip_ini,Tst2tip_ini2,endoPsi_ini]=getTandConfigV2;
[Tst2tip_ini,Tst2tip_ini2,endoPsi_ini,Tst2tc,TLm2marker]=getTandConfigV2;


Tst2tc(1:3,1)=cross(Tst2tc(1:3,2),Tst2tc(1:3,3))/norm(cross(Tst2tc(1:3,2),Tst2tc(1:3,3)));
Tst2tc(1:3,2)=cross(Tst2tc(1:3,3),Tst2tc(1:3,1))/norm(cross(Tst2tc(1:3,3),Tst2tc(1:3,1)));
TLm2marker(1:3,1)=cross(TLm2marker(1:3,2),TLm2marker(1:3,3))/norm(cross(TLm2marker(1:3,2),TLm2marker(1:3,3)));
TLm2marker(1:3,2)=cross(TLm2marker(1:3,3),TLm2marker(1:3,1))/norm(cross(TLm2marker(1:3,3),TLm2marker(1:3,1)));

x=[norm(Tst2tc(1:3,1)) norm(Tst2tc(1:3,2)) norm(Tst2tc(1:3,3)) 
    norm(Tst2tc(1,1:3)) norm(Tst2tc(2,1:3)) norm(Tst2tc(3,1:3)) 
    norm(TLm2marker(1:3,1)) norm(TLm2marker(1:3,2)) norm(TLm2marker(1:3,3)) 
    norm(TLm2marker(1,1:3)) norm(TLm2marker(2,1:3)) norm(TLm2marker(3,1:3))
    ];

options = optimoptions('fmincon','Algorithm','interior-point'); % 
options.StepTolerance=1e-25;
options.OptimalityTolerance=5e-6;





xN=zeros(1,10);
if(isSimplified==1)  
    xhmax=[   10        22.0     8        10.01     2 0 2*pi ];% 5 2];% -70 20 200];  
    xhmin=[   9         18        4.5     6.3     -2 -10 -2*pi ];%-5 -15 ];%-99 -20 160]; 
    xh0=[     10.0    19.4    20     5            10         -pi/0.4  0 -5 0];  %9.92
    xN(1)=99.2;
    Lsteam_Lr=391.5;
else
    xhmax=[   31         28.5     90         30         -20*pi/18  3 0];% 5 2];% -70 20 200];
    xhmin=[   29         10        40     5.0           -60*pi/18 -3 -10];%-5 -15 ];%-99 -20 160];
    xh0=[     30.0    19.4    20    50            10         -pi/0.4  0 -5];
    xN(1)=99.2;
    Lsteam_Lr=411.5;
end

xN(2:4)=xh0(1:3);
xN(5)=0.1;
xN(6)=xh0(4);xN(7)=xh0(5)/10;
xN(8)=381.5;xN(9)=xh0(6)/10;
xN(10)=0;
xNh=xN;

%% 通过投影优化gamma1

Pset_XY=zeros(size(endoPsi_ini,1),3);
Pset=zeros(size(endoPsi_ini,1),3);
Pset2=zeros(size(endoPsi_ini,1),3);
Pset_dev=zeros(size(endoPsi_ini,1),3);
dis_m1_m2=zeros(size(endoPsi_ini,1),1);
Pmean_set=zeros(5,3);
gamma1_set=zeros(5,1);
% figure;hold on;grid on;axis equal; 

% beta=-2.1708;
%    TLg2Lm=eye(4);TLm2marker1Position=eye(4);Tmarker1Position2YZ=eye(4);TYZ2Z=eye(4);
%     x_rot_ang=acos(dot([0 0 1]',TLm2marker_used(1:3,3)));
%     TLg2Lm(1:3,1:3)=[cos(xN(9)) -sin(xN(9)) 0 ;sin(xN(9)) cos(xN(9)) 0 ;0 0 1]';
%     
%     TLm2marker1Position(1:3,4)=TLm2marker_used(1:3,4);
%     Tmarker1Position2YZ(1:3,1:3)=[cos(beta) -sin(beta) 0 ;sin(beta) cos(beta) 0 ;0 0 1];
%     TYZ2Z(1:3,1:3)=[1 0 0;0 cos(x_rot_ang) -sin(x_rot_ang);0 sin(x_rot_ang) cos(x_rot_ang) ];
%     
    
for i =1:size(endoPsi_ini,1)
    temP_camera=reshape(Tst2tip_ini(1:3,4,i), [3 1]);
    temP2_camera=reshape(Tst2tip_ini2(1:3,4,i), [3 1]);
    Ttc2st=inv(Tst2tc);
    Pset_XY(i,:)=reshape(Ttc2st(1:3,1:3)*temP_camera+Ttc2st(1:3,4), [3 1]);
    Pset(i,:)=Pset_XY(i,:);
    Pset2(i,:)=reshape(Ttc2st(1:3,1:3)*temP2_camera+Ttc2st(1:3,4), [3 1]);
    Pset_dev(i,:)=Pset2(i,:)-Pset_XY(i,:);
    dis_m1_m2(i)=norm(Pset_dev(i,:));
%     plot3('*');
    Pset_XY(i,3)=0;
%     plot(Pset_XY(i,1),Pset_XY(i,2),'*');
end

Pmean_set(1,:)=mean(Pset_XY(1:5:end,:));
Pmean_set(2,:)=mean(Pset_XY(2:5:end,:));
Pmean_set(3,:)=mean(Pset_XY(3:5:end,:));
Pmean_set(4,:)=mean(Pset_XY(4:5:end,:));
Pmean_set(5,:)=mean(Pset_XY(5:5:end,:));
% plot(Pmean_set([2 4],1),Pmean_set([2 4],2),'-');
% plot(Pmean_set([3 5],1),Pmean_set([3 5],2),'-');
% plot(Pmean_set([2 1],1),Pmean_set([2 1],2),'-');
% plot(Pmean_set([3 1],1),Pmean_set([3 1],2),'-');
% plot(Pmean_set([1 4],1),Pmean_set([1 4],2),'-');
% plot(Pmean_set([1 5],1),Pmean_set([1 5],2),'-');
gamma1_set(1)=atan2(Pmean_set(1,2),Pmean_set(1,1));
gamma1_set(2)=atan2(Pmean_set(2,2),Pmean_set(2,1));
gamma1_set(3)=atan2(-Pmean_set(3,1),Pmean_set(3,2));
gamma1_set(4)=atan2(-Pmean_set(4,2),-Pmean_set(4,1));
gamma1_set(5)=atan2(Pmean_set(5,1),-Pmean_set(5,2))+10*pi/180;

gamma1=mean(gamma1_set(2:5))-pi/4-pi/4;

% 验证两点间距，间接验证相机精度是否准，
% 相差0.5204mm
% figure;stem(dis_m1_m2-mean(dis_m1_m2));
% max(dis_m1_m2)-min(dis_m1_m2);

% 验证
S24=Pmean_set(2,:)-Pmean_set(4,:);
S35=Pmean_set(3,:)-Pmean_set(5,:);
angle_24_35=acosd(dot(S24/norm(S24),S35/norm(S35)));

S23=Pmean_set(2,:)-Pmean_set(3,:);
S45=Pmean_set(4,:)-Pmean_set(5,:);
angle_23_45=acosd(dot(S23/norm(S23),S45/norm(S45)));

%% for show result
iterTimes=5;
initVal=zeros(iterTimes,size(xhmax,2));
finalVal=zeros(iterTimes,size(xhmax,2));
rVal=zeros(iterTimes,1);
exitFlag=zeros(iterTimes,1);
tic;
for i = 1:iterTimes
    OptSerial=zeros(3000,size(xhmax,2));
    residualVal=zeros(3000,1);
    optTimes=0;
    a=rand(1,size(xhmax,2)).*(xhmax-xhmin)+xhmin;
    initVal(i,:)=a;
% a=[9.7990   20.8055    4.9101    7.0431   -1.9896   -9.9799   -2.1681];
% a=[9.5854   21.2152    4.5640    9.7585    1.8086   -7.4154   -2.5866];
% a=[9.9869   21.9868    4.8447    6.0148   -1.1346   -9.9871    3.5789   -0.7560];
    [xh,yh,exitflag] = fmincon('costFuncVari_Final45_V05',a,[],[],[],[],xhmin,xhmax,[],options); %,options

    rV=residualVal;
    oS=OptSerial;
    rV(all(rV==0,2))=[];
    oS(rV>rV(end)*1.003,:)=[];
    oS=sortrows(oS,7,'ascend');
    oS(all(oS==0,2),:)=[];
    finalVal(i,:)=oS(end,:);
%     finalVal(i,:)=xh;
    rVal(i)=yh;
    exitFlag(i)=exitflag;
    
end
toc;
rVal1=rVal;
rVal1(exitFlag(:)==0,:)=[];
finalVal1=finalVal;
finalVal1(exitFlag(:)==0,:)=[];
[~,po]=min(rVal1);
XH11=finalVal1(po,:);

XH1_7=[XH11 [9.92 0.1].*[10 0.1]];
%% 优化L1和ζ
if(isSimplified==1)
    xhmax_v3=[10.4 2 ]; %200];
    xhmin_v3=[ 9.72 0.1 ];% 160];
else
    xhmax_v3=[85 2];
    xhmin_v3=[ 75 0.1];
end
iterTimes=5;
initVal_L1_zeta=zeros(iterTimes,size(xhmax_v3,2));
finalVal_L1_zeta=zeros(iterTimes,size(xhmax_v3,2));
rVal_L1_zeta=zeros(iterTimes,1);
exitFlag_L1_zeta=zeros(iterTimes,1);
tic;
for i = 1:iterTimes
    OptSerial=zeros(3000,size(xhmax_v3,2));
    residualVal=zeros(3000,1);
    optTimes=0;
    a=rand(1,size(xhmax_v3,2)).*(xhmax_v3-xhmin_v3)+xhmin_v3;
    initVal_L1_zeta(i,:)=a;
%     a=[9.8867    0.7980];
%     a=[10.1526    0.9470];
[xh2,yh2,exitflag] = fmincon('costFuncVari_Final23_V05',a,[],[],[],[],xhmin_v3,xhmax_v3,[],options); %,options
    finalVal_L1_zeta(i,:)=xh2;
    rVal_L1_zeta(i)=yh2;
    exitFlag_L1_zeta(i)=exitflag;
    rV=residualVal;
    oS=OptSerial;
    rV(all(rV==0,2))=[];
    oS(rV>rV(end)*1.003,:)=[];
    oS=sortrows(oS,1,'ascend');
    oS(all(oS==0,2),:)=[];
    finalVal_L1_zeta(i,:)=oS(1,:);
end
rVal1=rVal_L1_zeta;
rVal1(exitFlag_L1_zeta(:)==0,:)=[];
finalVal_L1_zeta1=finalVal_L1_zeta;
finalVal_L1_zeta1(exitFlag_L1_zeta(:)==0,:)=[];
[~,po]=min(rVal1);
XH22=finalVal_L1_zeta1(po,:);

XH1_7=[XH11 XH22.*[10 0.1]];


%%
% xh2=[XH11(9) XH11(8)];
% L1 Lr L2 Lg zeta K1 K2 Lstem gamma1  $L1x$ d beta 
XH=XH11;
xN=zeros(1,10);
xN(1)=XH22(1)*10;
xN(2)=XH(1);
xN(3)=19.4;
xN(4)=XH(2);
xN(5)=XH22(2)/10;
xN(6)=XH(3);xN(7)=XH(4)/10;
xN(8)=Lsteam_Lr-XH(1)+xNh(1)-xh2(1)*10;
xN(9)=gamma1;
xN(10)=XH(6);
xN(11)=XH1_7(7);
% 
fileConfig = fopen('../conf/matlab/finalPara.log','w');
fprintf(fileConfig,'%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n',xN);
fclose(fileConfig);
fpath=['../conf/configurationData/arm',num2str(path),'/finalPara.log'];
fileConfig = fopen(fpath,'w');
fprintf(fileConfig,'%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n',xN);
fclose(fileConfig);
%% 
[finalValue, validateConfig]=getPrecisionTargetV4;
fileConfig = fopen('../conf/psiData/testPrecision.log','w');
fileConfig2 = fopen('../conf/matlab/validateConfig.log','w');
for i =1:size(finalValue,1)
    fprintf(fileConfig,'%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f \n',finalValue(i,:));
end
for i =1:size(validateConfig,3)
    quat=rotm2quat(validateConfig(1:3,1:3,i));
    pose=validateConfig(1:3,4,i);
    temT=[pose' quat];
    fprintf(fileConfig2,'%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n',temT);
end
fclose(fileConfig);
fclose(fileConfig2);

fpath=['../conf/configurationData/arm',num2str(path),'/testPrecision.log'];
fileConfig = fopen(fpath,'w');
fpath=['../conf/configurationData/arm',num2str(path),'/validateConfig.log'];
fileConfig2 = fopen(fpath,'w');
for i =1:size(finalValue,1)
    fprintf(fileConfig,'%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f \n',finalValue(i,:));
end
for i =1:size(validateConfig,3)
    quat=rotm2quat(validateConfig(1:3,1:3,i));
    pose=validateConfig(1:3,4,i);
    temT=[pose' quat];
    fprintf(fileConfig2,'%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n',temT);
end
fclose(fileConfig);
fclose(fileConfig2);