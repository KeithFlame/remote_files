function J = Jacobian_differential_keith(psi,SL)
%JACOBIAN_DIFFERENTIAL_KEITH 此处显示有关此函数的摘要
%   此处显示详细说明



    T = FKcc_2segs_bending_keith(psi,SL);
    x0 = tem_T2x(T);
    block_size1 = size(psi,1);
    block_size2 = size(x0,1);
    J = zeros(block_size1,block_size2);
    dPsi = eye(block_size1)*1e-10;
    lambda = 1e-10;
    for i = 1:block_size1
        T = FKcc_2segs_bending_keith(psi+dPsi(:,i),SL);
        x = tem_T2x(T);
        nx = norm(x-x0);
        if(nx == 0)
            J(:,i) = x-x0;
        else
            J(:,i) = (x-x0)/1e-10;
        end
    end
    J = eye(block_size2,block_size2)/(J'*J+lambda*eye(block_size2,block_size2))*J';
end



