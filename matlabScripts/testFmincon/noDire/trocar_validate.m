clear all,clc;

global Ttt_trocar;
global psi0;

xhmax=[10 10 10 pi pi pi];
xhmin=[-10 -10 -10 -pi -pi -pi];
options = optimoptions('fmincon','Algorithm','interior-point','display','iter','PlotFcn','optimplotfval',"EnableFeasibilityMode",true); % 'Display','iter',,'PlotFcn','optimplotfirstorderopt','PlotFcn','optimplotfval'
options.StepTolerance=1e-25;
options.OptimalityTolerance=1e-6;
x=(xhmax+xhmin);
x=[0.2912    0.7238   -0.5322    0.0126   -0.0184    0.0009];
[xh,yh,exitFlag_k]=fmincon('costFunc_trocar',x,[],[],[],[],xhmin,xhmax,[],options);

p=xh(1:3)/1000;
zyx=xh(4:6);

R=eul2rotm(zyx);
Ttt_trocar=[R,p';[0 0 0 1]];
Ttt_trocar=eye(4);
%% init value
xhmax0=[pi 15 pi/2 pi pi/2 pi];
xhmin0=[-pi -15 -1e-6 -pi -1e-6 -pi];
num=1;
initV=zeros(size(xhmax0,2),num);
result_k=zeros(size(xhmax0,2),num);
exitflag_k0_b=zeros(1,num);
residual_k0=zeros(1,num);
x0=zeros(max(size(xhmax0)),1);
% x0=[0.0054    0.4993    0.0289    0.0021   -0.0000    0.0035]';  % 仅本身
% x0=[0.1187    0.0009    0.0237   -0.1255    0.0000   -0.1184]'; % 加上base_dev
% x0=[0.0055    0.0006    0.0259    0.0046   -0.0000    0.0013]';
for i =1 :num
%     x0=rand(size(xhmax0,2),1).*(xhmax0'-xhmin0')+xhmin0';
    [xh0,yh0,exitFlag_k0]=fmincon('costFunc_initValue',x0,[],[],[],[],xhmin0,xhmax0,[],options);
    residual_k0(i)=yh0;
    exitflag_k0_b(i)=exitFlag_k0;
    result_k(:,i)=xh0;
    initV(:,i)=x0;

end
xh0(2)=xh0(2)/1000;
psi0=xh0';



%% init + trocar
xhmax1=[10 10 10 pi pi pi];
xhmin1=[-10 -10 -10 -pi -pi -pi];
x1=(xhmax1+xhmin1)/2;
x1=[0.0575   -0.3201   -0.4780    0.0014   -0.0093   -0.0040];
[xh1,yh1,exitFlag_k1]=fmincon('costFunc_init_trocar',x1,[],[],[],[],xhmin1,xhmax1,[],options);