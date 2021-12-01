clear,clc;
%%
SP.psi.l=123.5*1e-3; % l>30
SP.psi.phi=0;
SP.psi.theta1=0.522;
SP.psi.delta1=0;
SP.psi.theta2=pi/2-SP.psi.theta1;
SP.psi.delta2=0;

SP=setInitValV1(SP);


theta1=SP.psi.theta1;
xhmax=[10 40 10 10];
xhmin=[-1e-9 -1e-9 -1e-9 0];
options = optimoptions('fmincon','Algorithm','interior-point','display','iter',"EnableFeasibilityMode",true); % 'Display','iter',,'PlotFcn','optimplotfirstorderopt','PlotFcn','optimplotfval'
options.StepTolerance=1e-25;
options.OptimalityTolerance=5e-6;
x=(xhmin+xhmax)/2;
x=rand(1,size(xhmin,2)).*(xhmax-xhmin)+xhmin;
% x=[0.0135    0.0258    0.0049    0.0449].*[100 100 300 200];
[xh,yh,exitflag] = fmincon('costFunc_u_Straight',x,[],[],[],[],xhmin,xhmax,'geth',options); %,options
xh=xh./[100 100 300 200];

figure;
Tc=plotResult(1,"");
Tn_d=plotResult_noClearance(1);
Tn=plotResult_noClearance(1,0);
SP=getSPV1;
tt=[SP.trocar.d exitflag]