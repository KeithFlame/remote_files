function [P1_far,P2_far,P3_far,P4_far]=plotVisionAuxillaryLine(T)
%% const para
    fov = 90/180*pi;
%     ratio = 16/9;
    mid1 = sqrt(16*16+9*9);
    mid2 = mid1/tan(fov/2);
    hfov = atan(16/mid2)*2;
    vfov = atan(9/mid2)*2;
    proximal = 30;
    distal = 100;
%% 
    T_tracker=T;
    P_tracker = T_tracker(1:3,4);

    R_tracker = T_tracker(1:3,1:3)*rotTest(0,0,0);
    right = R_tracker(1:3,1);
    up = R_tracker(1:3,2);
    front = R_tracker(1:3,3);


    P_front_axis_end = P_tracker + front*10;
    P_right_axis_end = P_tracker + right*10;
    P_up_axis_end = P_tracker + up*10;
    % pz_near = 30*front+P_tracker;
    % up_lim_near = 30 * tan(vfov/2);
    % right_lim_near = 30 * tan(hfov/2);
    P1_near = R_tracker*([proximal * tan(hfov/2);proximal * tan(vfov/2);proximal])+P_tracker;
    P1_far = R_tracker*([distal * tan(hfov/2);distal * tan(vfov/2);distal])+P_tracker;
    P2_near = R_tracker*([-proximal * tan(hfov/2);proximal * tan(vfov/2);proximal])+P_tracker;
    P2_far = R_tracker*([-distal * tan(hfov/2);distal * tan(vfov/2);distal])+P_tracker;
    P3_near = R_tracker*([-proximal * tan(hfov/2);-proximal * tan(vfov/2);proximal])+P_tracker;
    P3_far = R_tracker*([-distal * tan(hfov/2);-distal * tan(vfov/2);distal])+P_tracker;
    P4_near = R_tracker*([proximal * tan(hfov/2);-proximal * tan(vfov/2);proximal])+P_tracker;
    P4_far = R_tracker*([distal * tan(hfov/2);-distal * tan(vfov/2);distal])+P_tracker;

    % view([-180 -0]);
    plot3(P_tracker(1),P_tracker(2),P_tracker(3),'c*');
    line([P_tracker(1) P_front_axis_end(1)],[P_tracker(2) P_front_axis_end(2)],[P_tracker(3) P_front_axis_end(3)],'Color','b','LineWidth',1);
    line([P_tracker(1) P_right_axis_end(1)],[P_tracker(2) P_right_axis_end(2)],[P_tracker(3) P_right_axis_end(3)],'Color','r','LineWidth',1);
    line([P_tracker(1) P_up_axis_end(1)],[P_tracker(2) P_up_axis_end(2)],[P_tracker(3) P_up_axis_end(3)],'Color','g','LineWidth',1);

    line([0 0],[0 0],[0 20],'Color','b','LineWidth',2);
    line([0 0],[0 20],[0 0],'Color','g','LineWidth',2);
    line([0 20],[0 0],[0 0],'Color','r','LineWidth',2);

    line([P1_near(1) P2_near(1)],[P1_near(2) P2_near(2)],[P1_near(3) P2_near(3)],'Color','m','LineStyle','--');
    line([P2_near(1) P3_near(1)],[P2_near(2) P3_near(2)],[P2_near(3) P3_near(3)],'Color','m','LineStyle','--');
    line([P3_near(1) P4_near(1)],[P3_near(2) P4_near(2)],[P3_near(3) P4_near(3)],'Color','m','LineStyle','--');
    line([P4_near(1) P1_near(1)],[P4_near(2) P1_near(2)],[P4_near(3) P1_near(3)],'Color','m','LineStyle','--');

    line([P1_far(1) P2_far(1)],[P1_far(2) P2_far(2)],[P1_far(3) P2_far(3)],'Color','m','LineStyle','--');
    line([P2_far(1) P3_far(1)],[P2_far(2) P3_far(2)],[P2_far(3) P3_far(3)],'Color','m','LineStyle','--');
    line([P3_far(1) P4_far(1)],[P3_far(2) P4_far(2)],[P3_far(3) P4_far(3)],'Color','m','LineStyle','--');
    line([P4_far(1) P1_far(1)],[P4_far(2) P1_far(2)],[P4_far(3) P1_far(3)],'Color','m','LineStyle','--');

    line([P1_near(1) P1_far(1)],[P1_near(2) P1_far(2)],[P1_near(3) P1_far(3)],'Color','m','LineStyle','--');
    line([P2_near(1) P2_far(1)],[P2_near(2) P2_far(2)],[P2_near(3) P2_far(3)],'Color','m','LineStyle','--');
    line([P3_near(1) P3_far(1)],[P3_near(2) P3_far(2)],[P3_near(3) P3_far(3)],'Color','m','LineStyle','--');
    line([P4_near(1) P4_far(1)],[P4_near(2) P4_far(2)],[P4_near(3) P4_far(3)],'Color','m','LineStyle','--');
end