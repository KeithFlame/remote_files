clear,clc;
%%
SP.l=45.5*1e-3; % l>30


SP.phi=0;
SP.theta1=0.5616;
SP.delta1=0;
SP.theta2=pi/2-SP.theta1;
SP.delta2=0;

SP=setInitVal(SP);


theta1=SP.theta1;
xhmax=[10 40 10 10];
xhmin=[-1e-9 -1e-9 -1e-9 0];
options = optimoptions('fmincon','Algorithm','interior-point','display','iter',"EnableFeasibilityMode",true); % 'Display','iter',,'PlotFcn','optimplotfirstorderopt','PlotFcn','optimplotfval'
options.StepTolerance=1e-25;
% options.OptimalityTolerance=5e-6;
x=(xhmin+xhmax)/2;
x=rand(1,size(xhmin,2)).*(xhmax-xhmin)+xhmin;
[xh,yh,exitflag] = fmincon('costFunc_u_V2',x,[],[],[],[],xhmin,xhmax,'geth',options); %,options
xh=xh./[100 100 300 200];
if(xh(1)<1e-6)
    SP.d=xh(3);
else
    SP.d=xh(4);
end
if(SP.Ls>1e-5)
    SP.thetasi=abs(xh(1));
    SP.thetaso=xh(2);
    SP.theta1i=0;
    SP.theta1o=SP.theta1-SP.thetasi-SP.thetaso-SP.theta1i;
    SP.Lsi=SP.d/sin(SP.thetasi)*SP.thetasi;
    SP.L1i=0;
    SP.Lso=SP.Ls-SP.Lsi+SP.d;
    SP.L1o=SP.L1;
else
    SP.thetaso=0;
    SP.Lso=0;
    SP.theta1i=xh(2);
    if(xh(4)-xh(3)<1e-5)
        SP.thetasi=0;
        SP.Lsi=0;  
        SP.theta1o=SP.theta1-SP.thetasi-SP.thetaso-SP.theta1i;
        SP.L1i=SP.d/sin(SP.theta1i)*SP.theta1i;
        SP.L1o=SP.l-SP.Lr-SP.L2-SP.L1i+SP.d;
        
    else
        SP.thetasi=xh(1);
        dsi=SP.d-xh(3);
        SP.Lsi=dsi/sin(SP.thetasi)*SP.thetasi;
        SP.L1i=xh(3)/cos(SP.thetasi)/sin(SP.theta1i)*SP.theta1i;
        SP.theta1o=SP.theta1-SP.thetasi-SP.thetaso-SP.theta1i;
        SP.L1o=SP.l-SP.Lr-SP.L2-SP.L1i+xh(3)/cos(SP.thetasi);
        
    end
end

setInitVal(SP);
figure;
Tc=plotResult(1,"");
% Tn=plotResult_noClearance(1);
Tn=plotResult_noClearance(1,0);
ttt=[SP.d,SP.L1i,SP.thetasi+SP.theta1i,exitflag]