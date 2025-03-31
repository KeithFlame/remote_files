function p = getMiddleT(cur_T,tar_T,incremental)
    p_err = norm(cur_T(1:3,4)-tar_T(1:3,4));
    % 将旋转矩阵转换为四元数
    q1 = rotm2quat(cur_T(1:3,1:3));
    q2 = rotm2quat(tar_T(1:3,1:3));
    
    % 计算插值的数量
    dot_product = dot(q1, q2);
    if dot_product < 0
        q2 = -q2;
        dot_product = -dot_product;
    end
    theta_0 = acosd(dot_product);
    flag = 0;
        if(abs(theta_0)>incremental)
            t = incremental / 180*pi;
            q_interp = slerp(q1, q2, t); 
            rot = quat2rotm(q_interp);
        else
            rot = tar_T(1:3,1:3);
            flag = 1;
        end
        if(p_err>incremental)
            % if(flag == 1)
            %     incremental = incremental*2;
            % end
            pos = -(cur_T(1:3,4)-tar_T(1:3,4))/p_err*incremental+cur_T(1:3,4);
        else
            pos = tar_T(1:3,4);
        end
    p = eye(4);
    p(1:3,1:3)=rot;p(1:3,4)=pos;

end

function q_interp = slerp(q1, q2, t)
    % 球面线性插值函数
    dot_product = dot(q1, q2);
    
    % 修正角度
    if dot_product < 0
        q2 = -q2;
        dot_product = -dot_product;
    end
    
    if dot_product > 0.9995
        % 线性插值
        q_interp = q1 + t * (q2 - q1);
        q_interp = q_interp / norm(q_interp);
        return;
    end

    theta_0 = acos(dot_product);
    sin_theta_0 = sin(theta_0);
    theta = theta_0 * t;
    sin_theta = sin(theta);

    s1 = sin_theta / sin_theta_0;
    s2 = sin(theta_0 * (1 - t)) / sin_theta_0;

    q_interp = s1 * q1 + s2 * q2;
    q_interp = q_interp / norm(q_interp);
end