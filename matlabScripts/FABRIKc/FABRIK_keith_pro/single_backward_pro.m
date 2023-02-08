function mu = single_backward_pro(motion_units,i,hd,is_plot)
%MOTION_UNIT 此处显示有关此函数的摘要
%   此处显示详细说明
%   运动单元建立
    u1x = motion_units(i - 1).e2;
    u1z = motion_units(i - 1).de2;
    motion_units(i).joint_position = motion_units(i - 1).end_position + ...
        u1x * motion_units(i).l1;
    if_cond = max(size(motion_units))*0;
    if((i+1) < if_cond && (motion_units(i+1).joint_type == 0 || ...
            motion_units(i+1).joint_type == 4))
        u2x = -(motion_units(i+2).joint_position - motion_units(i).joint_position)/...
        norm(motion_units(i+2).joint_position - motion_units(i).joint_position);
    elseif((i+2) < if_cond && (motion_units(i+1).joint_type == 0 || ...
            motion_units(i+1).joint_type == 4)&& (motion_units(i+2).joint_type == 0 || ...
            motion_units(i+2).joint_type == 4))
        u2x = -(motion_units(i+3).joint_position - motion_units(i).joint_position)/...
        norm(motion_units(i+3).joint_position - motion_units(i).joint_position);
    else
        u2x = -(motion_units(i+1).joint_position - motion_units(i).joint_position)/...
            norm(motion_units(i+1).joint_position - motion_units(i).joint_position);
    end
    motion_units(i).origin_position = motion_units(i-1).end_position;
    motion_units(i).origin_direction_position = motion_units(i-1).end_direction_position;

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

%% 计算phi对末端方向的影响
    phi = motion_units(i).phi;
    assert(isreal(phi));
    u1y = cross(u1z,u1x); % ee1 x;eeb1 y;
    u1y = u1y/norm(u1y);

%% go on
    switch (joint_type)  %   0 刚体运动单元    FMU
        case 0
            theta = motion_units(i).theta;
            if(theta<1e-6)
                u2x = -u1x;
                u2z = u1z*cos(phi)-u1y*sin(phi);
            else
                delta = motion_units(i).delta;
                u1y = cross(u1z,u1x);
                u1y = u1y / norm(u1y);
                e_delta = u1z*cos(delta) - u1y*sin(delta);
                u2x = -(u1x*cos(theta) + e_delta * sin(theta));
                e_delta_theta = - sin(theta) * u1x + cos(theta) * e_delta;
                tem_y = cross(-u2x,e_delta_theta);
                e_delta_theta_delta = cos(-delta) * e_delta_theta + sin(-delta) * tem_y;
                u2y_delta_theta = cross(-u2x, e_delta_theta_delta);
                u2y_delta_theta = u2y_delta_theta / norm(u2y_delta_theta);
                u2z = cos(phi) * e_delta_theta_delta + sin(phi) * u2y_delta_theta;
            end
            u2x = u2x / norm(u2x);
            u2z = u2z / norm(u2z);
            motion_units(i).end_position = motion_units(i).joint_position - ...
                    u2x * motion_units(i).l2;
            motion_units(i).end_direction_position = motion_units(i).end_position + ...
                motion_units(i).l2 * u2z * motion_units(i).lateral_ratio;

        case 1 %   1 移动运动单元    PMU
            l1 = - norm(motion_units(i+1).joint_position - ...
            motion_units(i-1).end_position) * u2x'*u1x - motion_units(i).l2 ...
             - motion_units(i+1).l1;
            if(l1>motion_units(i).constraint_limit.l1_lim(2))
                l1 = motion_units(i).constraint_limit.l1_lim(2);
            elseif(l1<motion_units(i).constraint_limit.l1_lim(1))
                l1=motion_units(i).constraint_limit.l1_lim(1);
            end
            motion_units(i).joint_position = motion_units(i - 1).end_position + ...
                    u1x * l1;
            motion_units(i).end_position = motion_units(i).joint_position + ...
                motion_units(i).l2 * u1x;
            e_phi = cos(phi) * u1z - sin(phi) * u1y;
            motion_units(i).end_direction_position = motion_units(i).end_position + e_phi *...
                motion_units(i).l2 * motion_units(i).lateral_ratio;
            
        case 2 %   2 万向节运动单元   UMU
            theta = acos(-u2x'*u1x);
            theta_lim = motion_units(i).constraint_limit.theta_lim(2);
            if(theta>theta_lim)
                theta = theta_lim;
                ee3 = cross(-u2x,u1x);
                ee4 = cross(u1x,ee3);ee4 = ee4 / norm(ee4);
                ee3 = ee4*sin(theta_lim) + u1x*cos(theta_lim);
                ee3 = ee3/norm(ee3);
                u2x = - ee3;
                motion_units(i).end_position = motion_units(i).joint_position - ...
                    motion_units(i).l2 * u2x;
                
            else
                motion_units(i).end_position = motion_units(i).joint_position - ...
                    motion_units(i).l2 * u2x;
                
            end
            % theta, delta, phi
            dot_mu2x_u1y = motion_units(i).setDotValue(-u2x, u1y);
            dot_mu2x_u1z = motion_units(i).setDotValue(-u2x, u1z);
            if(abs(dot_mu2x_u1z)<1e-6 && abs(dot_mu2x_u1y)<1e-6)
                delta = 0;
            else
                delta = atan2(-dot_mu2x_u1y,dot_mu2x_u1z);
            end
            
            e_delta = cos(delta) * u1z - sin(delta) * u1y;
            
            if(theta < 1e-6)
                e_delta_theta_phi = cos(phi) * u1z - sin(phi) * u1y;
            else
                e_delta_theta = - sin(theta) * u1x + cos(theta) * e_delta;
                tem_y = cross(-u2x,e_delta_theta);
                tem_y = tem_y / norm(tem_y);
                e_delta_theta_delta = cos(-delta) * e_delta_theta + sin(-delta) * tem_y;
                u2y_delta_theta = cross(-u2x, e_delta_theta_delta);
                u2y_delta_theta = u2y_delta_theta / norm(u2y_delta_theta);
                e_delta_theta_phi = cos(phi) * e_delta_theta_delta + sin(phi) * u2y_delta_theta;
            end
            motion_units(i).end_direction_position = motion_units(i).end_position + ...
                    motion_units(i).l2 * e_delta_theta_phi * motion_units(i).lateral_ratio;

        case 3 %   3 连续体运动单元   CMU
            if(motion_units(i).theta < 1e-6)
                L = motion_units(i).l1 + motion_units(i).l2;
            else
                L = motion_units(i).theta * motion_units(i).l1/tan(motion_units(i).theta/2);
            end
            theta = acos(-u2x'*u1x);
            theta_lim = motion_units(i).constraint_limit.theta_lim(2);
            if (theta > theta_lim)
                theta = theta_lim;
                ee3 = cross(-u2x,u1x);
                ee4 = cross(u1x,ee3);ee4 = ee4 / norm(ee4);
                ee3 = ee4*sin(theta_lim) + u1x*cos(theta_lim);
                ee3 = ee3/norm(ee3);
                u2x = - ee3;
            end
            if(theta <1e-6)
                motion_units(i).l1 = L/2;
                motion_units(i).l2 = L/2;
            else
                motion_units(i).l1 = L/theta*tan(theta/2);
                motion_units(i).l2 = motion_units(i).l1;
            end
            motion_units(i).joint_position = motion_units(i).origin_position ...
                 + motion_units(i).l1 * u1x;
            motion_units(i).end_position = motion_units(i).joint_position ...
                - motion_units(i).l2 *u2x;
            
            % theta, delta, phi
            dot_mu2x_u1y = motion_units(i).setDotValue(-u2x, u1y);
            dot_mu2x_u1z = motion_units(i).setDotValue(-u2x, u1z);
            delta = atan2(-dot_mu2x_u1y,dot_mu2x_u1z);
            
            e_delta = cos(delta) * u1z - sin(delta) * u1y;
            
            if(theta < 1e-6)
                e_delta_theta_phi = cos(phi) * u1z - sin(phi) * u1y;
            else
                e_delta_theta = - sin(theta) * u1x + cos(theta) * e_delta;
                tem_y = cross(-u2x,e_delta_theta);
                tem_y = tem_y / norm(tem_y);
                e_delta_theta_delta = cos(-delta) * e_delta_theta + sin(-delta) * tem_y;
                u2y_delta_theta = cross(-u2x, e_delta_theta_delta);
                u2y_delta_theta = u2y_delta_theta / norm(u2y_delta_theta);
                e_delta_theta_phi = cos(phi) * e_delta_theta_delta + sin(phi) * u2y_delta_theta;
            end
            motion_units(i).end_direction_position = motion_units(i).end_position + ...
                    motion_units(i).l2 * e_delta_theta_phi * motion_units(i).lateral_ratio;

        case 4 %   4 轴向旋转运动单元    PiMU
            
            motion_units(i).end_position = motion_units(i).joint_position + ...
                motion_units(i).l2 * u1x;
            tv = motion_units(i+1).de1;
            u2y = cross(tv,-u1x);
            
            if(sum(abs(u2y))<1e-6)
                u2z = -motion_units(i+1).e1;
            else
                u2z = cross(-u1x,u2y);
                u2z = u2z / norm(u2z);
            end
            motion_units(i).end_direction_position = motion_units(i).end_position + ...
                motion_units(i).l2 * u2z * motion_units(i).lateral_ratio;
            

            

        case 5 %   5 径向旋转运动单元  RMU

            u1y = cross(u1z, u1x);
            u2z = cos(phi) * u1z - sin(phi) * u1y;
            u2z = u2z / norm(u2z);
            aa = motion_units(i).joint_position(1);
            ab = motion_units(i).joint_position(2);
            ac = motion_units(i).joint_position(3);
            ax0 = motion_units(i + 1).joint_position(1);
            ay0 = motion_units(i + 1).joint_position(2);
            az0 = motion_units(i + 1).joint_position(3);
            am = u2z(1);an = u2z(2);as = u2z(3);
            at = -(am*(ax0-aa) + an*(ay0-ab) + as*(az0-ac))/(am*am + an*an + as*as);
            ax = am*at + ax0;ay = an*at + ay0;az = as*at +az0;
            tar_p = [ax ay az]';
            
            % 正式计算另外两个位置变量
            u2x = (tar_p - motion_units(i).joint_position)/...
            norm(tar_p - motion_units(i).joint_position);
            theta = acos(u2x'*u1x);
            theta_lim = motion_units(i).constraint_limit.theta_lim(2);
            if(theta>theta_lim)
                ee3 = cross(u2x,u1x);
                ee4 = cross(u1x,ee3);
                ee3 = ee4/norm(ee4)*sin(theta_lim) + u1x*cos(theta_lim);
                ee3 = ee3/norm(ee3);
                motion_units(i).end_position = motion_units(i).joint_position + ...
                    motion_units(i).l2 * ee3;
            else
                motion_units(i).end_position = motion_units(i).joint_position + ...
                    motion_units(i).l2 * u2x;
                
            end
            motion_units(i).end_direction_position = motion_units(i).end_position + ...
                motion_units(i).l2 * motion_units(i).lateral_ratio * u2z;

            
        case 6 %   6 球铰运动单元    SMU
            theta = acos(-u2x'*u1x);
            theta_lim = motion_units(i).constraint_limit.theta_lim(2);
            if(theta>theta_lim)
                ee3 = cross(-u2x,u1x);
                ee4 = cross(u1x,ee3);
                ee3 = ee4/norm(ee4)*sin(theta_lim) + u1x*cos(theta_lim);
                ee3 = ee3/norm(ee3);
                u2x = - ee3;
                motion_units(i).end_position = motion_units(i).joint_position - ...
                    motion_units(i).l2 * u2x;      
            else
                motion_units(i).end_position = motion_units(i).joint_position - ...
                    motion_units(i).l2 * u2x;         
            end

            tv = motion_units(i+1).de1;
            u2y = cross(tv,u2x);
            u2z = cross(u2x,u2y);
            u2z = u2z / norm(u2z);
            motion_units(i).end_direction_position = motion_units(i).end_position + ...
            motion_units(i).l2 * u2z * motion_units(i).lateral_ratio;

        case 7 % 备用 

    end
    
    if(is_plot)
        motion_units(i).is_plot=is_plot;
        mu = motion_units(i).updateUnit;
        pause(0.1);
    else
        mu = motion_units(i).refreshUnit;
    end
end
