SP.l=40.1*1e-3; % l>30


SP.phi=0;
SP.theta1=0.5616;
SP.delta1=0;
SP.theta2=pi/2-SP.theta1;
SP.delta2=0;

SP=setInitVal(SP);

% x=[0.1    0.1    0.1    0.19];

theta1=SP.theta1;
xhmax=[10 40  10];
xhmin=[-1e-9 -1e-9  0.01];
options = optimoptions('fmincon','Algorithm','interior-point','display','iter',"EnableFeasibilityMode",true); % 'Display','iter',,'PlotFcn','optimplotfirstorderopt','PlotFcn','optimplotfval'
options.StepTolerance=1e-25;
% options.OptimalityTolerance=5e-6;
x=(xhmin+xhmax)/2;
x=rand(1,size(xhmin,2)).*(xhmax-xhmin)+xhmin;
% x=[0.0230    0.000000001    0.0162 ].*[100 100 200];
[xh,yh,exitflag] = fmincon('costFunc_u_V3',x,[],[],[],[],xhmin,xhmax,'geth_V3',options); %,options
xh=xh./[100 100 200];

SP=setFinalSP(xh);
figure;
Tc=plotResult(1,"");
% Tn=plotResult_noClearance(1);
Tn=plotResult_noClearance(1,0);
ttt=[SP.d,SP.L1i,SP.thetasi+SP.theta1i,exitflag]