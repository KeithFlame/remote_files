function [error_angle,error_axis] = error_orientation(orientation_current,orientation_target)
%   ���ݵ�ǰ��̬��Ŀ����̬�����̬ƫ���-�Ǳ�ʾ��
    error_R=orientation_target/orientation_current;
    error_angle=acos((trace(error_R)-1)/2);
    error_axis=1/(2*sin(error_angle))*[error_R(3,2)-error_R(2,3);error_R(1,3)-error_R(3,1);error_R(2,1)-error_R(1,2)];

end

