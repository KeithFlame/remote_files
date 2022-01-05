function OP=setOP

persistent OP0;
if(isempty(OP0))
    OP0.xhmin1=[-1e-3 -1e-3 -1e-3 -1e-3 -1e-3 -1e-3];
    OP0.xhmax1=[2 2 2 2 2 2];
    OP0.x1=(OP0.xhmin1+OP0.xhmax1)/2;
    OP0.xhmin2=[-1e-3 -1e-3 -1e-3 -1e-3 -1e-3];
    OP0.xhmax2=[2 2 2 2 2 ];
    OP0.x2=(OP0.xhmin2+OP0.xhmax2)/2;
    OP0.xhmin3=[-1e-3 -1e-3 -1e-3 -1e-3 -1e-3 -1e-3 -1e-3];
    OP0.xhmax3=[2 2 2 2 2 2 2];
    OP0.x3=(OP0.xhmin3+OP0.xhmax3)/2;
    OP0.options = optimoptions('fmincon','Algorithm','interior-point',"EnableFeasibilityMode",true);
    OP0.options = optimoptions('fmincon','Algorithm','interior-point',"EnableFeasibilityMode",true); % 'Display','iter',,'PlotFcn','optimplotfirstorderopt','PlotFcn','optimplotfval'
    OP0.options.StepTolerance=1e-25;
    OP0.options.OptimalityTolerance=5e-6;

end
OP=OP0;
end