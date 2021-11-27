% x0=0;
% options = optimoptions('fmincon','Display','iter','Algorithm','interior-point'); 
% 
% [x,fval,exitflag,output] = fmincon('costFuncGamma3',x0,[],[],[],[],[],[],[],options);
% x,x0,
% gamma3=x;

xh0=[18 8 30 19 10 10 15 53.1 5*pi/1.8];
% xh0=[18.9981    8.1698   29.3257   18.7633    9.5581   17.1543   15.6277    5.4459  -7.44017];
xhmax=[20 8.5 32 20 15 15 15.01 55.0 20*pi/1.8];
xhmin=[16 7 28.5 17 0.1 0.1 14.99 14.99 -20*pi/1.8];
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point'); 

options.StepTolerance=1e-15;
options.OptimalityTolerance=1e-10;
options.MaxFunctionEvaluations=3e4;
options.FunctionTolerance=1e-10;
options.MaxIterations=1e4;
% options.
% nonlcon = @unitdisk;
A=[
-1 -1 -1 -1 0 0 0 0 0
1 1 1 1 0 0 0 0 0
-1 -1 -1 0 0 0 0 0 0
1 1 1 0 0 0 0 0 0
% 0 0 1 0 0 0 0 0 0
% 0 0 0 1 0 0 0 0 0
% 0 0 0 0 0 0 0 1 0
];
b=[-72;77;-54;57]; % ;9;21;7
% xh0=[18.1577    7.7923   29.6322   19.6955   14.9301    9.4326   15.5717    6.0767    6.9085];
[xh,yh] = fmincon('costFunc',xh0,A,b,[],[],xhmin,xhmax,[],options);
xh
xh(5) = xh(5) / 10;
xh(6) = xh(6) / 10;
xh(8) = xh(8) * 10
c=sum(xh(1:4))
% fsolve('costFuncStructure',xh0,options);

arm_id_num = load([getenv('VSARMCALIBPATH'), '\', 'arm_id.log']);
result_name = join([getenv('VSARMCALIBPATH'), '\conf\',  'opt_result.log']);
result_archive_name = join([getenv('VSARMCALIBPATH'), '\result\', num2str(arm_id_num), '\opt_result.log']);
dlmwrite(result_name, xh, 'precision', '%.4f');
dlmwrite(result_archive_name, xh, 'precision', '%.4f');