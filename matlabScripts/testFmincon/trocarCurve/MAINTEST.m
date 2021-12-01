clear,clc;

SP.psi.l=140.5*1e-3; % l>30
SP.psi.phi=0;
SP.psi.theta1=0.99;
SP.psi.delta1=0;
SP.psi.theta2=pi/2-SP.psi.theta1;
SP.psi.delta2=0;

SP=setInitVal(SP);
% x=[0.1    0.1    0.1    0.19];

theta1=SP.psi.theta1;
xhmax=[10 40 10 10 20];
xhmin=[-1e-9 -1e-9 0 0 20];
options = optimoptions('fmincon','Algorithm','interior-point','display','iter',"EnableFeasibilityMode",true); % 'Display','iter',,'PlotFcn','optimplotfirstorderopt','PlotFcn','optimplotfval'
options.StepTolerance=1e-25;
% options.OptimalityTolerance=5e-6;
x=(xhmin+xhmax)/2;
x=rand(1,size(xhmin,2)).*(xhmax-xhmin)+xhmin;
getSP(x);
[xh,yh,exitflag] = fmincon('costFunc_u',x,[],[],[],[],xhmin,xhmax,'getGH',options); %,options
% xh=xh./[100 100 100 100 100];

SP=getSP;
figure;
Tc=plotResult(1,"");
% Tn=plotResult_noClearance(1);
% Tn=plotResult_noClearance(1,0);
ttt=[SP.dependent_psi.l1i,SP.dependent_psi.thetasi+SP.dependent_psi.theta1i,exitflag]