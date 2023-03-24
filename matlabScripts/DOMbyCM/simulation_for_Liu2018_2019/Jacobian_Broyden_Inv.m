function J = Jacobian_Broyden_Inv(J_1,dPsi,dZ)
%JACOBIAN_FRIDEN 此处显示有关此函数的摘要
%   此处显示详细说明

    alpha = 0.8;
%     [m,n] = size(J_1);
%     J_1 = eye(n,m)/J_;
    J = J_1 + alpha * (dPsi-J_1*dZ)/(dPsi'*J_1*dZ)*dPsi'*J_1;

end

