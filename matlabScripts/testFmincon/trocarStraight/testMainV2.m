clear,clc;
%%
SP.psi.l=0.1057; % l>30
SP.psi.phi=4.71;
SP.psi.theta1=pi/2;
SP.psi.delta1=pi/2;
SP.psi.theta2=1.7058;
SP.psi.delta2=-1.2328;

SP=setInitValV1(SP);


theta1=SP.psi.theta1;
xhmax=[10 40 10 10];
xhmin=[-1e-3 -1e-3 -1e-3 0];
options = optimoptions('fmincon','Algorithm','interior-point','display','iter',"EnableFeasibilityMode",true); % 'Display','iter',,'PlotFcn','optimplotfirstorderopt','PlotFcn','optimplotfval'
options.StepTolerance=1e-25;
options.OptimalityTolerance=5e-6;
x=(xhmin+xhmax)/2;
x=rand(1,size(xhmin,2)).*(xhmax-xhmin)+xhmin;
% x=[0    0    0.0049    0.0449].*[100 100 300 200];
[xh,yh,exitflag] = fmincon('costFunc_u_Straight',x,[],[],[],[],xhmin,xhmax,'geth',options); %,options
xh=xh./[100 100 300 200];

figure;
Tc=plotResult(1,"");
% Tn_d=plotResult_noClearance(1);
Tn=plotResult_noClearance(0,0);
SP=getSPV1;
axis([-5e-3 +5e-3 -5e-3 +5e-3 -25e-3 +5e-3])
% axis([Tc(1,4)-5e-6 Tc(1,4)+5e-6 Tc(2,4)-5e-6 Tc(2,4)+5e-6 Tc(3,4)-5e-6 Tc(3,4)+5e-6])

tt=[SP.trocar.d exitflag ]
Tn(1:3,4)'-Tc(1:3,4)'