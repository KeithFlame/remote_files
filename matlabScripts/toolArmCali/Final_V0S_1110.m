% Ver. 1.8
% Author：Keith
% K1是变量，d是变量，L1x是变量,L2是定值 加入了L1和ζ
% Date：13.43 08.16.2021
clear;
clc;
global Tst2tip_ini;
global endoPsi_ini;
global qa_actual;
global XH1_7;
global xN_init;

global OptSerial;
global optTimes;
global residualVal;
residualVal=zeros(3000,1);
optTimes=0;

serials='70_4';
[Tst2tip_ini,endoPsi_ini,qa_actual,xN_init]=getTandConfigV2_1110(serials);
isSimplified=1;

    
Tst2tc= eye(4);

Tst2tc(1:3,1)=cross(Tst2tc(1:3,2),Tst2tc(1:3,3))/norm(cross(Tst2tc(1:3,2),Tst2tc(1:3,3)));
Tst2tc(1:3,2)=cross(Tst2tc(1:3,3),Tst2tc(1:3,1))/norm(cross(Tst2tc(1:3,3),Tst2tc(1:3,1)));

x=[norm(Tst2tc(1:3,1)) norm(Tst2tc(1:3,2)) norm(Tst2tc(1:3,3)) 
    norm(Tst2tc(1,1:3)) norm(Tst2tc(2,1:3)) norm(Tst2tc(3,1:3)) ];

options = optimoptions('fmincon','Display','iter','Algorithm','interior-point','PlotFcn','optimplotfirstorderopt','PlotFcn','optimplotfval'); % 
options.StepTolerance=1e-25;
options.OptimalityTolerance=5e-6;





xN=zeros(1,10);
if(isSimplified==1)  
    xhmax=[   10        22.0     7        12         80*pi/18  5 0 10.4 2];
    xhmin=[   9         10        1     1.0           -60*pi/18 -10 -20 9.71 0.1]; 
    xh0=[     10.0    19.4    20     5            10         -pi/0.4  0 -5 ];  %9.92
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

%% for show result
iterTimes=3;
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
%     optimal_path=['../systemCali/test1102/optimal_res_',serials,'.mat'];
%     t=load(optimal_path);
%     optimal_res=t.optimal_res;
%     a=optimal_res(1:11);
%     a=[optimal_res(2) optimal_res(4) optimal_res(6)...
%         optimal_res(7)*10 optimal_res(9)*10 optimal_res(10) optimal_res(11)...
%          optimal_res(1)/10  optimal_res(5)*10];

    [xh,yh,exitflag] = fmincon('costFuncVari_Final45_V0S_1110',a,[],[],[],[],xhmin,xhmax,[],options); %,options

    rV=residualVal;
    oS=OptSerial;
    rV(all(rV==0,2))=[];
    oS(rV>rV(end)*1.000003,:)=[];
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

XH1_7=XH11;
%% 优化L1和ζ
% % if(isSimplified==1)
% %     xhmax_v3=[10.4 2 ]; %200];
% %     xhmin_v3=[ 9.72 0.1];% 160];
% % else
% %     xhmax_v3=[85 2];
% %     xhmin_v3=[ 75 0.1];
% % end
% % iterTimes=3;
% % initVal_L1_zeta=zeros(iterTimes,size(xhmax_v3,2));
% % finalVal_L1_zeta=zeros(iterTimes,size(xhmax_v3,2));
% % rVal_L1_zeta=zeros(iterTimes,1);
% % exitFlag_L1_zeta=zeros(iterTimes,1);
% % tic;
% % for i = 1:iterTimes
% %     a=rand(1,size(xhmax_v3,2)).*(xhmax_v3-xhmin_v3)+xhmin_v3;
% %     initVal_L1_zeta(i,:)=a;
% % %     a=[ 10.0795    0.1004];
% % [xh2,yh2,exitflag] = fmincon('costFuncVari_Final23_V0S_1110',a,[],[],[],[],xhmin_v3,xhmax_v3,[],options); %,options
% %     finalVal_L1_zeta(i,:)=xh2;
% %     rVal_L1_zeta(i)=yh2;
% %     exitFlag_L1_zeta(i)=exitflag;
% %     
% % end

%%

xh2=[XH11(8) XH11(9)];
XH=XH11;
xN=zeros(1,11);
xN(1)=xh2(1)*10;
xN(2)=XH(1);
xN(3)=19.4;
xN(4)=XH(2);
xN(5)=xh2(2)/10;
xN(6)=XH(3);xN(7)=XH(4)/10;
xN(8)=Lsteam_Lr-XH(1);
xN(9)=0*XH(5)/10;
xN(10)=XH(6);
xN(11)=XH(7);
optimal_res=xN;
fname=['../systemCali/test1102/optimal_res_nogamma1_',serials];
save(fname,'optimal_res');
