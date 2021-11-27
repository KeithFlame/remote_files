x= [ 0         0   -0.1775   -0.2944    0.2813    0.6731];
A=[0 0 1 0 0 0;
   0 0 -1 0 0 0;
   0 0 0 1 0 0;
   0 0 0 -1 0 0];
b=[pi/4;pi/4;pi/2;pi/2];

options = optimoptions('fmincon','Display','iter','Algorithm','interior-point'); 

[x,y]=fmincon('getJointValue',x,A,b,[],[],[],[],[],options);
% 
% LS=[17.2241    8.3457   30.6355   15.8762];
% gamma3=-6.9327/100;
% Rgamma3=rotTest(0 ,0, gamma3);
% P1tc=[0 0 sum(LS)]'+Rgamma3*[-4.59803, 8.70767, 104.899]';


verify_cfg_file = join([getenv('VSARMCALIBPATH'), '\conf\', 'opt_result.log']);
opt_result = load(verify_cfg_file);
LS = opt_result(1:4);
gamma3 = -opt_result(end) / 100;
Rgamma3=rotTest(0 ,0, gamma3);
outfilename = join([getenv('VSARMCALIBPATH'), '\calib_pic_coord_out.log']);
M_raw = importdata(outfilename);
M = str_rep(M_raw);
P1tc0 = M(1, 13:15);
P1tc = [0 0 sum(LS)]' + Rgamma3*P1tc0';

% x=[0 0 0 0 pi/4 pi/2];
% flag=[1 1];
% 
verify_cfg_idx = evalin('base', 'verify_cfg_idx');
flag_list=[
    1 1;
    1 -1;
    -1 -1;
    -1 1];
flag = flag_list(verify_cfg_idx, :);

config=[x(1) x(2) x(3) LS(1) x(4) LS(2) x(5) LS(3) x(6) LS(4)];
[Tc,~,~,~,~,~,s1,s2,s3,s4,s5]=FKfunc(config);
Tc(1:3,1:3)=Tc(1:3,1:3)*Rgamma3;

Tp1=[eye(3) P1tc;[0 0 0 1]];
if y<0.000001
    figure;hold on;xlabel('X');ylabel('Y');zlabel('Z');axis equal;view([45 20]);axis([ -200 200 -200 200 -0 400]);

    plotCoord(Tp1);
    plot3(s1(1,:),s1(2,:),s1(3,:),'k',s2(1,:),s2(2,:),s2(3,:),'r',s3(1,:),s3(2,:),s3(3,:),'g',s4(1,:),s4(2,:),s4(3,:),'b',s5(1,:),s5(2,:),s5(3,:),'m','linewidth',2,'LineStyle','-');
    plotVisionAuxillaryLine(Tc);
end
verify_cfg_param = x*180/pi

verify_cfg_file = join([getenv('VSARMCALIBPATH'), '\conf\', 'verify_cfg_', num2str(verify_cfg_idx), '.log']);
dlmwrite(verify_cfg_file, verify_cfg_param, 'precision', '%.4f');
