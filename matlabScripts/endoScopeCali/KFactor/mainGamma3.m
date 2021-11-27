
xh0=[18 8 30 19 15 12 0 ];
xhmax=[21.0 9 32 20 20 20 20*pi/1.8];
xhmin=[17.0 7 28 16 0.001 0.001 -20*pi/1.8];
% tX=[1e-6 -1e-6 -1e-6 1e-90 1 1 1];,'TypicalX',tX
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point','PlotFcn','optimplotfirstorderopt'); 

options.StepTolerance=1e-20;
options.OptimalityTolerance=1e-6;
options.MaxFunctionEvaluations=3e4;
options.FunctionTolerance=1e-10;
options.MaxIterations=1e4;

A=[
-1 -1 -1 -1 0 0 0
1 1 1 1 0 0 0
% 1 1 1 0 0 0 0
% -1 -1 -1 0 0 0 0
% 0 1 1 1 0 0 0
% 0 -1 -1 -1 0 0 0
% 0 0 1 1 0 0 0
% 0 0 -1 -1 0 0 0
];
b=[-70;80]; % ;9;21;7;57;-53.5156;57;-55;49;-46.4873
[xh,yh,exitflag,output] = fmincon('costFunc',xh0,A,b,[],[],xhmin,xhmax,[],options);
xh
c=sum(xh(1:4))
