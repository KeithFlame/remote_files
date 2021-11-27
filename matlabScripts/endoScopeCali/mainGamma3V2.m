%%
global Cf;
global Tt;
global Res;
[Cf,Tt] = getConfigandT;

%%

xh0=[18 8 30 19 10 10 pi/0.4 53.1 -0];
% xh0=[17.6573    7.2580   28.6055   19.2876   11.2026   13.9134    7.9788   63.3324   -7.6821];
xhmax=[20 9 32 20 15 15 pi/0.35 65.0 10*pi/1.8];
xhmin=[17 7 28 16 6.1 0.1 pi/0.45 50.0 -10*pi/1.8];
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
% -1 -1 -1 0 0 0 0 0 0
% 1 1 1 0 0 0 0 0 0
% 0 0 1 0 0 0 0 0 0
% 0 0 0 1 0 0 0 0 0
% 0 0 0 0 0 0 0 1 0
];
b=[-72;78]; % ;9;21;7;-54;57
% xh0=[18.1577    7.7923   29.6322   19.6955   14.9301    9.4326   15.5717    6.0767    6.9085];
[xh,yh,exitflag,output] = fmincon('costFuncV2',xh0,A,b,[],[],xhmin,xhmax,[],options);

c=sum(xh(1:4));
%%
Res=xh;
[xh2,yh2,exitflag,output] = fmincon('getGamma_a',-100,[],[],[],[],[],[],[],options);  %-1.57;0  1;-1

xN=xh;xN(5)=xh(5)/10;xN(6)=xh(6)/10;xN(8)=xh(8)*10;xN(9)=-xh(9);
xN(7)=-xh2
arm_id_num = load([getenv('VSARMCALIBPATH'), '\', 'arm_id.log']);
result_name = join([getenv('VSARMCALIBPATH'), '\conf\',  'opt_result.log']);
result_archive_name = join([getenv('VSARMCALIBPATH'), '\result\', num2str(arm_id_num), '\opt_result.log']);
dlmwrite(result_name, xN, 'precision', '%.4f');
dlmwrite(result_archive_name, xN, 'precision', '%.4f');
validate = (xh2+xN(end)/100)*180/pi
verify_phi_cfg_name = join([getenv('VSARMCALIBPATH'), '\conf\',  'verify_phi_cfg.log']);
verify_phi_cfg = [0, validate, 0, 0, 0, 0];
dlmwrite(verify_phi_cfg_name, verify_phi_cfg, 'precision', '%.4f');
jointlimttest_cfg_name1 = join([getenv('VSARMCALIBPATH'), '\conf\',  'jointlimttest_cfg1.log']);
jointlimttest_cfg1 = [0, validate, 0, 0, 0, 0];
dlmwrite(jointlimttest_cfg_name1, jointlimttest_cfg1, 'precision', '%.4f');
jointlimttest_cfg_name2 = join([getenv('VSARMCALIBPATH'), '\conf\',  'jointlimttest_cfg2.log']);
jointlimttest_cfg2 = [0, validate, 45, 0, 0, 0];
dlmwrite(jointlimttest_cfg_name2, jointlimttest_cfg2, 'precision', '%.4f');
jointlimttest_cfg_name3 = join([getenv('VSARMCALIBPATH'), '\conf\',  'jointlimttest_cfg3.log']);
jointlimttest_cfg3 = [0, validate, 0, 0, 90, 0];
dlmwrite(jointlimttest_cfg_name3, jointlimttest_cfg3, 'precision', '%.4f');