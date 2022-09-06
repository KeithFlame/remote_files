function mu = single_backward(motion_units,i,hd,is_plot)
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

    if(is_plot)
        P = [motion_units(i+1).joint_position motion_units(i).joint_position ...
            motion_units(i-1).joint_position];
        set(hd(1),'XData',P(1,:),'YData',P(2,:),'ZData',P(3,:));
        set(hd(2),'XData',P(1,2),'YData',P(2,2),'ZData',P(3,2));
        pause(0.1);
    end

    if(motion_units(i).joint_type == 0)  %刚性段
        motion_units(i).end_position = motion_units(i).joint_position+ ...
        ee1 * motion_units(i).l2;
    elseif(motion_units(i).joint_type == 1)  % 移动副
%         if(ee2'*ee1<0)
%             l1 = 1e-6;
%         else
            l1 = norm(motion_units(i+1).joint_position - ...
            motion_units(i-1).end_position) * ee2'*ee1 - motion_units(i).l2 ...
             - motion_units(i+1).l1;
%         end

        if(l1>motion_units(i).constraint_limit.l1_lim(2))
            l1 = motion_units(i).constraint_limit.l1_lim(2);
        elseif(l1<motion_units(i).constraint_limit.l1_lim(1))
            l1=motion_units(i).constraint_limit.l1_lim(1);
        end

        motion_units(i).joint_position = motion_units(i - 1).end_position + ...
                ee1 * l1;
        motion_units(i).end_position = motion_units(i).joint_position + ...
            motion_units(i).l2 * ee1;
    elseif(motion_units(i).joint_type == 2) % 万向节
        motion_units(i).end_position = motion_units(i).joint_position + ...
            motion_units(i).l2 * ee2;
    elseif(motion_units(i).joint_type == 3) % 连续体副
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
    end
    
    if(is_plot)
        motion_units(i).is_plot=is_plot;
        mu = motion_units(i).updateUnit;
        pause(0.1);
    else
        mu = motion_units(i).refreshUnit;
    end
end

