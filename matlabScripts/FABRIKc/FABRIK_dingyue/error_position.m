function [e_P_scalar,e_P_unite] = error_position(position_current,position_target)
%   根据当前位置和目标位置和求解位置偏差
    e_P_vector=position_target-position_current;
    e_P_scalar=norm(e_P_vector);
    e_P_unite=e_P_vector/e_P_scalar;
end

