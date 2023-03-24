function J = Jacobian_Broyden(J_,dPsi,dZ)
%JACOBIAN_FRIDEN 此处显示有关此函数的摘要
%   此处显示详细说明

    alpha = 0.99;
     
    t= alpha * (dZ - J_ * dPsi) / (dPsi'*dPsi)*dPsi';
    J = J_ + t;
end

