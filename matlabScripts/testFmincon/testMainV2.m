SP.l=150*1e-3; % l>30


SP.phi=0;
SP.theta1=pi/4;
SP.delta1=0;
SP.theta2=pi/2-SP.theta1;
SP.delta2=0;

SP=setInitVal(SP);
% x=[0.1    0.1    0.1    0.19];

theta1=SP.theta1;
xhmax=[theta1 theta1 0.1 0.1];
xhmin=[-1e-9 -1e-9 -1e-9 0];
options = optimoptions('fmincon','Algorithm','interior-point',"EnableFeasibilityMode",true); % 'Display','iter',,'PlotFcn','optimplotfirstorderopt','PlotFcn','optimplotfval'
options.StepTolerance=1e-25;
x=[0.0008    0.0002    0.0502    0.1503];
[xh,yh,exitflag] = fmincon('costFunc_u',x,[],[],[],[],xhmin,xhmax,'geth',options); %,options

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
Tc=plotResult(1,"clearance=1e-3");
% Tn=plotResult_noClearance(1);
Tn=plotResult_noClearance(1,0);