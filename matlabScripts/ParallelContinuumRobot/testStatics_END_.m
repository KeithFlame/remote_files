% show one arm.
%
% Author: Keith W.
% Ver. 1.0
% Date 07.29.2022

clear all;
SP = getStructurePara_keith;
init_pose = SP.init_pose;
% figure;
view([90 0]);axis equal;grid on; hold on;
trocar = SP.trocar;
plotTrocar(trocar);

arm1 = getArmPara(112,1);
arm2 = getArmPara(1112,1);
gamma1 = 0;
% psi= [40 45 90 0 0 0
%     40 45 0 0 120 0
%     70.825  0    93.7244   90.000   48.7243  -90.000
%    70.825  0    93.7244   0.000   48.7243  -180.000
%    70.825  0    93.7244   -90.000   48.7243  90.000
%    70.825  0    93.7244   180.000   48.7243  0.000];
psi=[58 0 0 0 0 0 0
    104.843    0    41.6394    90    41.6394    -90 0
    104.843    0    41.6394    180    41.6394    0 0
      42.0775    0    46.8103    -90    1.8103    90 0
      42.0775    0    46.8103    0    1.8103    180 0];
psi(:,2) = psi(:,2) +gamma1; 
% psi = [psi(1,:) 0.7854];
% psi = [80 0 0 0 0 0 1];
psi(:,2:6) = psi(:,2:6)*pi/180;
%%
Fe=[-0 -0. -0.]';
Me=[0 0 0]';
fe=[0 0 0]';
le=[0 0 0]';
external_ = [Fe;Me;fe;le];
%%

% [t,u,res]=forwardStatic_END_andPlotSnake_keith(psi(1,:),arm1,external_,1);
forwardKinematicsandPlotSnake_keith(psi(1,:),arm1,1);
forwardKinematicsandPlotSnake_keith(psi(2,:),arm1,1);
forwardKinematicsandPlotSnake_keith(psi(3,:),arm1,1);
forwardKinematicsandPlotSnake_keith(psi(4,:),arm1,1);
forwardKinematicsandPlotSnake_keith(psi(5,:),arm1,1);
xlabel("X (mm)");
ylabel("Y (mm)");
zlabel("Z (mm)");
set(gca,fontsize = 14);
view([-225 25])

%% 应变结果
% ep11 = norm(res(7:8));
% ep21 = norm(res(9:10));
% 
% u0 = sqrt(u(1,:).^2 + u(2,:).^2);
% ep12 = 0.00095/2/(1/max(u0(1:42))-0.0025);
% ep22 = 0.00040/2/(1/max(u0)-0.0027) *cosd(21);
% ep1 = [ep11 ep12 ep11+ep12]*100;
% ep2 = [ep21 ep22 ep21+ep22]*100;
% ep1 = roundn(ep1,-3), ep2 = roundn(ep2,-3), 
% mean(0.00040/2./(1./u0-0.0027) * cosd(21)),