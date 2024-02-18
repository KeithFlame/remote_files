% this is a function to calculate inverse kinematics of a 7-dof robot.

%% priori knowledge
p_base = [58 -572.5+50 -85]';
O_RCM=[0 -232.5+100 100]';
R_base = eul2rotm([0 0 pi]);
T_base = [R_base p_base;[0 0 0 1]];
[arm, tool] = trolleyInfo;
pose = getWorkspaceTargets(arm.d8);
Ttar = fromX2T(pose(:,1100));

%% execution
% [Qc,flag,iter] = invKine_7DoF(T_base,Ttar,arm);
[Qce,flage,itere,Jace] = invKine_7DoF_inEnd(T_base,Ttar,arm);

% [~,~,~,~,~,~,~,~,~,~,~,~,~,~,Tbase7,~]=FK_7DOF_Offset(arm,T_base,Qc);
% [~,~,~,~,~,~,~,~,~,~,~,~,~,~,Tbase7_,~]=FK_7DOF_Offset(arm,T_base,Qce);
[Jac, Ni] = vertexJacobian_7DoF_inEnd(T_base,arm,Qce);
Jvw = Jace(3:5,:);
% JJ1 = Ni*Jac*pinv(Jvw);
% sv1 = svd(JJ1);
JJ2 = Ni(1:18,1:18)*Jac(1:18,:)*pinv(Jvw);
sv2 = svd(JJ2);
I1 = sv2(1);
I3 = sv2(3)/sv2(1);
sv2 (all(sv2 == 0, 2),:) = [];
I2 = prod(sv2);
cter = [I1 I2 I3]';

%% plot
[Arm_7DOF_Offset,Tbase7]=Plot_Arm7_DOF_Offset(arm,tool,T_base,Qce);
grid on; hold on; axis equal;
PlotAxis(100,[Expm([0 0 0]') [0 0 0]';0 0 0 1]);
PlotAxis(100,[Expm([0 0 0]') O_RCM;0 0 0 1]);
PlotAxis(200,Tbase7);