function [e_P_scalar,e_P_unite] = error_position(position_current,position_target)
%   ���ݵ�ǰλ�ú�Ŀ��λ�ú����λ��ƫ��
    e_P_vector=position_target-position_current;
    e_P_scalar=norm(e_P_vector);
    e_P_unite=e_P_vector/e_P_scalar;
end

