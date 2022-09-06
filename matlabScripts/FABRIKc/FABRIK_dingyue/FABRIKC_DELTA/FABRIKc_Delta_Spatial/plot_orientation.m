function plot_orientation(position,orientation,length)
%   ������ά��ͷ�Ա�ʾ
%   position ��ͷ���
%   oriention ��̬����

    
    orientation_x=length*orientation(:,1);
    orientation_y=length*orientation(:,2);
    orientation_z=length*orientation(:,3);
    plot3([position(1),position(1)+orientation_x(1)],[position(2),position(2)+orientation_x(2)],[position(3),position(3)+orientation_x(3)],'r','linewidth',1);
    hold on
    plot3([position(1),position(1)+orientation_y(1)],[position(2),position(2)+orientation_y(2)],[position(3),position(3)+orientation_y(3)],'g','linewidth',1);
    
    plot3([position(1),position(1)+orientation_z(1)],[position(2),position(2)+orientation_z(2)],[position(3),position(3)+orientation_z(3)],'b','linewidth',1);
    
end

