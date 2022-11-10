function mu = single_backward_pro(motion_units,i,hd,is_plot)
%MOTION_UNIT 此处显示有关此函数的摘要
%   此处显示详细说明
%   运动单元建立
    ee1 = motion_units(i - 1).e2;
    motion_units(i).joint_position = motion_units(i - 1).end_position + ...
        ee1 * motion_units(i).l1;
    lens = max(size(motion_units));
    if((i+1) ~= lens && motion_units(i+1).joint_type == 0)
        ee2 = (motion_units(i+2).joint_position - motion_units(i).joint_position)/...
        norm(motion_units(i+2).joint_position - motion_units(i).joint_position);
    else
        ee2 = (motion_units(i+1).joint_position - motion_units(i).joint_position)/...
            norm(motion_units(i+1).joint_position - motion_units(i).joint_position);
    end
    motion_units(i).origin_position = motion_units(i-1).end_position;
%% plot line
    if(is_plot)
        P = [motion_units(i+1).joint_position motion_units(i).joint_position ...
            motion_units(i-1).joint_position];
        set(hd(1),'XData',P(1,:),'YData',P(2,:),'ZData',P(3,:));
        set(hd(2),'XData',P(1,2),'YData',P(2,2),'ZData',P(3,2));
        pause(0.1);
    end
%% joint type declarition
    joint_type = motion_units(i).joint_type;
    if(joint_type == 5 && (motion_units(i+1).joint_type == 6 || ...
            motion_units(i-1).joint_type == 6))
        joint_type = 2;
    end
%% go on
    switch (joint_type)  %   0 刚体运动单元    FMU
        case 0
            motion_units(i).end_position = motion_units(i).joint_position+ ...
            ee1 * motion_units(i).l2;

        case 1 %   1 移动运动单元    PMU
            l1 = norm(motion_units(i+1).joint_position - ...
            motion_units(i-1).end_position) * ee2'*ee1 - motion_units(i).l2 ...
             - motion_units(i+1).l1;
            if(l1>motion_units(i).constraint_limit.l1_lim(2))
                l1 = motion_units(i).constraint_limit.l1_lim(2);
            elseif(l1<motion_units(i).constraint_limit.l1_lim(1))
                l1=motion_units(i).constraint_limit.l1_lim(1);
            end
            motion_units(i).joint_position = motion_units(i - 1).end_position + ...
                    ee1 * l1;
            motion_units(i).end_position = motion_units(i).joint_position + ...
                motion_units(i).l2 * ee1;
            
        case 2 %   2 万向节运动单元   UMU
            theta = acos(ee2'*ee1);
            if(theta>motion_units(i).constraint_limit.theta_lim)
                ee3 = cross(ee2,ee1);
                ee4 = cross(ee1,ee3);
                ee3 = ee4/norm(ee3)/sqrt(3) + ee1;
                ee3 = ee3/norm(ee3);
                motion_units(i).end_position = motion_units(i).joint_position + ...
                    motion_units(i).l2 * ee3;
            else
                motion_units(i).end_position = motion_units(i).joint_position + ...
                    motion_units(i).l2 * ee2;
            end

        case 3 %   3 连续体运动单元   CMU
            if(motion_units(i).theta < 1e-6)
                L = motion_units(i).l1 + motion_units(i).l2;
            else
                L = motion_units(i).theta * motion_units(i).l1/tan(motion_units(i).theta/2);
            end
            theta = acos(ee2'*ee1);
            if (theta > motion_units(i).constraint_limit.theta_lim(2))
                theta_tem = theta;
                theta = motion_units(i).constraint_limit.theta_lim(2);
                tem = theta_tem - theta;
                dd = sin(tem)/sin(theta);
                eee2 = ee2 + dd*ee1;
                ee2 = eee2/norm(eee2);
            end
            if(theta <1e-6)
                motion_units(i).l1 = L/2;
                motion_units(i).l2 = L/2;
            else
                motion_units(i).l1 = L/theta*tan(theta/2);
                motion_units(i).l2 = motion_units(i).l1;
            end
            motion_units(i).joint_position = motion_units(i).origin_position ...
                 + motion_units(i).l1 * ee1;
            motion_units(i).end_position = motion_units(i).joint_position ...
                + motion_units(i).l2 *ee2;

        case 4 %   4 轴向旋转运动单元    PiMU
            ee3 = cross(ee2,ee1);
            eev2 = cross(ee1,ee3);
            eev2 = eev2/norm(eev2);
            theta = motion_units(i).theta;
            eee2 = cos(theta) * ee1 + sin(theta) * eev2;
            motion_units(i).end_position = motion_units(i).joint_position + ...
                    motion_units(i).l2 * eee2;

        case 5 %   5 径向旋转运动单元  RMU
            delta = motion_units(i).delta;


        case 6 %   6 球铰运动单元    SMU
            theta = acos(ee2'*ee1);
            if(theta>motion_units(i).constraint_limit.theta_lim)
                ee3 = cross(ee2,ee1);
                ee4 = cross(ee1,ee3);
                ee3 = ee4/norm(ee3)/sqrt(3) + ee1;
                ee3 = ee3/norm(ee3);
                motion_units(i).end_position = motion_units(i).joint_position + ...
                    motion_units(i).l2 * ee3;
            else
                motion_units(i).end_position = motion_units(i).joint_position + ...
                    motion_units(i).l2 * ee2;
            end

    end
    
    if(is_plot)
        motion_units(i).is_plot=is_plot;
        mu = motion_units(i).updateUnit;
        pause(0.1);
    else
        mu = motion_units(i).refreshUnit;
    end
end

