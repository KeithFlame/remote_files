function mu = single_forward_pro(motion_units,i,hd,is_plot,err_flag)
%MOTION_UNIT 此处显示有关此函数的摘要
%   此处显示详细说明
%   运动单元建立
    u2x = motion_units(i + 1).e1;
    u2z = motion_units(i + 1).de1;
    motion_units(i).joint_position = motion_units(i + 1).origin_position + ...
        u2x * motion_units(i).l2;
    if_cond = 100;
    if((i - 1) > if_cond &&  (motion_units(i-1).joint_type == 0 || ...
            motion_units(i-1).joint_type == 4))
        u1x = - (motion_units(i-2).joint_position - motion_units(i).joint_position)/...
        norm(motion_units(i-2).joint_position - motion_units(i).joint_position);
    elseif((i - 2) > if_cond &&  (motion_units(i-1).joint_type == 0 || ...
            motion_units(i-1).joint_type == 4) &&(motion_units(i-2).joint_type == 0 || ...
            motion_units(i-2).joint_type == 4))
        u1x = - (motion_units(i-3).joint_position - motion_units(i).joint_position)/...
        norm(motion_units(i-3).joint_position - motion_units(i).joint_position);
    else
        u1x = - (motion_units(i-1).joint_position - motion_units(i).joint_position)/...
            norm(motion_units(i-1).joint_position - motion_units(i).joint_position);
    end
    motion_units(i).end_position = motion_units(i+1).origin_position;
    motion_units(i).end_direction_position = motion_units(i+1).origin_direction_position;
    
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
    errp_cond = 0.1;
%% 计算phi对末端方向的影响
    phi = motion_units(i).phi;
    assert(isreal(phi));
    u2y = cross(u2z, u2x); 
    u2y = u2y/norm(u2y);
%% go on
    switch (joint_type)  %   0 刚体运动单元    FMU
        case 0
            
            theta = motion_units(i).theta;
            if(theta<1e-6)
                u1x0 = -u2x;
            else
                delta = phi - motion_units(i).delta;
                u2y = cross(u2z,u2x);
                u2y = u2y / norm(u2y);
                e_delta = u2z*cos(delta) - u2y*sin(delta);
                u1x0 = -(u2x*cos(theta) + e_delta * sin(theta));
            end
            u1x0 = u1x0 / norm(u1x0);
            muop = motion_units(i).joint_position - u1x0 * motion_units(i).l1;

            theta = acos(- u1x'*u2x);
            theta_lim = motion_units(i).constraint_limit.theta_lim(2);
            if(theta>theta_lim)
                ee3 = cross(- u1x,u2x);
                ee4 = cross(u2x,ee3);ee4 = ee4 / norm(ee4);
                ee3 = ee4 * sin(theta_lim) + u2x * cos(theta_lim);
                u1x = - ee3/norm(ee3);
                
                motion_units(i).origin_position = motion_units(i).joint_position - ...
                    motion_units(i).l1 * u1x;
            else
                motion_units(i).origin_position = motion_units(i).joint_position - ...
                    motion_units(i).l1 * u1x;
            end
            dt = norm((muop - motion_units(i).origin_position));
            if(err_flag || abs(dt - motion_units(i).err_recorded)>errp_cond)
                motion_units(i).err_flag = 0;
            else
                motion_units(i).err_flag = 1;

                u1x = u1x0;
                motion_units(i).origin_position = muop;
            end

            dot_mu1x_u2y = motion_units(i).setDotValue(-u1x, u2y);
            dot_mu1x_u2z = motion_units(i).setDotValue(-u1x, u2z);
            if(abs(dot_mu1x_u2z)<1e-6 && abs(dot_mu1x_u2y)<1e-6)
                delta = 0;
            else
                delta = atan2(-dot_mu1x_u2y,dot_mu1x_u2z);
            end
            
            e_delta = cos(delta) * u2z - sin(delta) * u2y;
            if(theta < 1e-6)
                e_delta_theta_phi = cos(phi) * u2z - sin(phi) * u2y;
            else
                e_delta_theta = - sin(theta) * u2x + cos(theta) * e_delta;
                tem_y = cross(-u1x,e_delta_theta);
                e_delta_theta_delta = cos(-delta) * e_delta_theta + sin(-delta) * tem_y;
                u1y_delta_theta = cross(-u1x, e_delta_theta_delta);
                u1y_delta_theta = u1y_delta_theta / norm(u1y_delta_theta);
                e_delta_theta_phi = cos(phi) * e_delta_theta_delta + sin(phi) * u1y_delta_theta;
            end
            motion_units(i).origin_direction_position = motion_units(i).origin_position + ...
                    motion_units(i).l2 * e_delta_theta_phi * motion_units(i).lateral_ratio;

            motion_units(i).err_recorded = dt;

            if(dt<errp_cond && abs(dt - motion_units(i).err_recorded)<errp_cond)
                motion_units(i).err_flag = 0;
            end


        case 1 %   1 移动运动单元    PMU
            l1 = - norm(motion_units(i-1).joint_position - ...
            motion_units(i).joint_position) * u1x'*u2x - motion_units(i-1).l2;
            if(l1>motion_units(i).constraint_limit.l1_lim(2))
                l1 = motion_units(i).constraint_limit.l1_lim(2);
            elseif(l1<motion_units(i).constraint_limit.l1_lim(1))
                l1 = motion_units(i).constraint_limit.l1_lim(1);
            end
            motion_units(i).origin_position = motion_units(i).joint_position + ...
                    u2x * l1;
            e_phi = cos(phi) * u2z - sin(phi) * u2y;
            motion_units(i).origin_direction_position = motion_units(i).origin_position + ...
                e_phi * motion_units(i).l2 * motion_units(i).lateral_ratio;
            
        case 2 %   2 万向节运动单元   UMU
            theta = acos(- u1x'*u2x);
            theta_lim = motion_units(i).constraint_limit.theta_lim(2);
            if(theta>theta_lim)
                ee3 = cross(- u1x,u2x);
                ee4 = cross(u2x,ee3);ee4 = ee4 / norm(ee4);
                ee3 = ee4 * sin(theta_lim) + u2x * cos(theta_lim);
                u1x = - ee3/norm(ee3);
                
                motion_units(i).origin_position = motion_units(i).joint_position - ...
                    motion_units(i).l1 * u1x;
            else
                motion_units(i).origin_position = motion_units(i).joint_position - ...
                    motion_units(i).l1 * u1x;
            end

            dot_mu1x_u2y = motion_units(i).setDotValue(-u1x, u2y);
            dot_mu1x_u2z = motion_units(i).setDotValue(-u1x, u2z);
            if(abs(dot_mu1x_u2z)<1e-6 && abs(dot_mu1x_u2y)<1e-6)
                delta = 0;
            else
                delta = atan2(-dot_mu1x_u2y,dot_mu1x_u2z);
            end
            
            e_delta = cos(delta) * u2z - sin(delta) * u2y;
            if(theta < 1e-6)
                e_delta_theta_phi = cos(phi) * u2z - sin(phi) * u2y;
            else
                e_delta_theta = - sin(theta) * u2x + cos(theta) * e_delta;
                tem_y = cross(-u1x,e_delta_theta);
                e_delta_theta_delta = cos(-delta) * e_delta_theta + sin(-delta) * tem_y;
                u1y_delta_theta = cross(-u1x, e_delta_theta_delta);
                u1y_delta_theta = u1y_delta_theta / norm(u1y_delta_theta);
                e_delta_theta_phi = cos(phi) * e_delta_theta_delta + sin(phi) * u1y_delta_theta;
            end
            motion_units(i).origin_direction_position = motion_units(i).origin_position + ...
                    motion_units(i).l2 * e_delta_theta_phi * motion_units(i).lateral_ratio;

        case 3 %   3 连续体运动单元   CMU
            if(motion_units(i).theta < 1e-6)
                L = motion_units(i).l1 + motion_units(i).l2;
            else
                L = motion_units(i).theta * motion_units(i).l1/tan(motion_units(i).theta/2);
            end
            theta = acos(- u1x'*u2x);
            theta_lim = motion_units(i).constraint_limit.theta_lim(2);
            if (theta > theta_lim)
                theta = theta_lim;
                ee3 = cross(- u1x,u2x);
                ee4 = cross(u2x,ee3);ee4 = ee4 / norm(ee4);
                ee3 = ee4 * sin(theta_lim) + u2x * cos(theta_lim);
                u1x = - ee3/norm(ee3);
            end
            if(theta <1e-6)
                motion_units(i).l1 = L/2;
                motion_units(i).l2 = L/2;
            else
                motion_units(i).l1 = L/theta*tan(theta/2);
                motion_units(i).l2 = motion_units(i).l1;
            end
            motion_units(i).joint_position = motion_units(i).end_position ...
                 + motion_units(i).l2 * u2x;
            motion_units(i).origin_position = motion_units(i).joint_position ...
                - motion_units(i).l1 *u1x;
        
            dot_mu1x_u2y = motion_units(i).setDotValue(-u1x, u2y);
            dot_mu1x_u2z = motion_units(i).setDotValue(-u1x, u2z);
            delta = atan2(-dot_mu1x_u2y,dot_mu1x_u2z);
            e_delta = cos(delta) * u2z - sin(delta) * u2y;
            if(theta < 1e-6)
                e_delta_theta_phi = cos(phi) * u2z - sin(phi) * u2y;
            else
                e_delta_theta = - sin(theta) * u2x + cos(theta) * e_delta;
                tem_y = cross(-u1x,e_delta_theta);
                e_delta_theta_delta = cos(-delta) * e_delta_theta + sin(-delta) * tem_y;
                u1y_delta_theta = cross(-u1x, e_delta_theta_delta);
                u1y_delta_theta = u1y_delta_theta / norm(u1y_delta_theta);
                e_delta_theta_phi = cos(phi) * e_delta_theta_delta + sin(phi) * u1y_delta_theta;
            end
            motion_units(i).origin_direction_position = motion_units(i).origin_position + ...
                    motion_units(i).l2 * e_delta_theta_phi * motion_units(i).lateral_ratio;

        case 4 %   4 轴向旋转运动单元    PiMU
            motion_units(i).origin_position = motion_units(i).joint_position + ...
                motion_units(i).l1 * u2x;
            tv = motion_units(i-1).de2;
            
            u1y = cross(tv,-u2x);
            if(sum(abs(u2y))<1e-6)
                u1z = -motion_units(i+1).e1;
            else
                u1z = cross(-u2x,u1y);
                u1z = u1z / norm(u1z);
            end
            motion_units(i).origin_direction_position = motion_units(i).origin_position + ...
                    motion_units(i).l2 * u1z * motion_units(i).lateral_ratio;

        case 5 %   5 径向旋转运动单元  RMU
            theta = acos(- u1x'*u2x);
            theta_lim = motion_units(i).constraint_limit.theta_lim(2);
            if(theta>theta_lim)
                ee3 = cross(- u1x,u2x);
                ee4 = cross(u2x,ee3);ee4 = ee4 / norm(ee4);
                ee3 = ee4 * sin(theta_lim) + u2x * cos(theta_lim);
                u1x = - ee3/norm(ee3);
                
                motion_units(i).origin_position = motion_units(i).joint_position - ...
                    motion_units(i).l1 * u1x;
            else
                motion_units(i).origin_position = motion_units(i).joint_position - ...
                    motion_units(i).l1 * u1x;
            end

            aa = motion_units(i).joint_position(1);
            ab = motion_units(i).joint_position(2);
            ac = motion_units(i).joint_position(3);
            ax0 = motion_units(i - 1).joint_position(1);
            ay0 = motion_units(i - 1).joint_position(2);
            az0 = motion_units(i - 1).joint_position(3);
            am = u2z(1);an = u2z(2);as = u2z(3);
            at = -(am*(ax0-aa) + an*(ay0-ab) + as*(az0-ac))/(am*am + an*an + as*as);
            ax = am*at + ax0;ay = an*at + ay0;az = as*at +az0;
            tar_p = [ax ay az]';
            
            % 正式计算另外两个位置变量
            u1x0 = -(tar_p - motion_units(i).joint_position)/...
            norm(tar_p - motion_units(i).joint_position);
            theta = acos(-u1x0'*u2x);
            theta_lim = motion_units(i).constraint_limit.theta_lim(2);
            if(theta>theta_lim)
                ee3 = cross(-u1x0,u2x);
                ee4 = cross(u2x,ee3);
                ee3 = ee4/norm(ee4)*sin(theta_lim) + u2x*cos(theta_lim);
                ee3 = ee3/norm(ee3);
                muop = motion_units(i).joint_position + ...
                    motion_units(i).l1 * ee3;
            else
                muop = motion_units(i).joint_position - ...
                    motion_units(i).l1 * u1x0;
            end

            dt = norm((muop - motion_units(i).origin_position));
            if(err_flag || abs(dt - motion_units(i).err_recorded)>errp_cond)
                motion_units(i).err_flag = 0;
            else
                motion_units(i).err_flag = 1;

                u1x = u1x0;
                motion_units(i).origin_position = muop;
            end

            dot_mu1x_u2y = motion_units(i).setDotValue(-u1x, u2y);
            dot_mu1x_u2z = motion_units(i).setDotValue(-u1x, u2z);
            if(abs(dot_mu1x_u2z)<1e-6 && abs(dot_mu1x_u2y)<1e-6)
                delta = 0;
            else
                delta = atan2(-dot_mu1x_u2y,dot_mu1x_u2z);
            end
            
            e_delta = cos(delta) * u2z - sin(delta) * u2y;
            if(theta < 1e-6)
                e_delta_theta_phi = cos(phi) * u2z - sin(phi) * u2y;
            else
                e_delta_theta = - sin(theta) * u2x + cos(theta) * e_delta;
                tem_y = cross(-u1x,e_delta_theta);
                e_delta_theta_delta = cos(-delta) * e_delta_theta + sin(-delta) * tem_y;
                u1y_delta_theta = cross(-u1x, e_delta_theta_delta);
                u1y_delta_theta = u1y_delta_theta / norm(u1y_delta_theta);
                e_delta_theta_phi = cos(phi) * e_delta_theta_delta + sin(phi) * u1y_delta_theta;
            end
            motion_units(i).origin_direction_position = motion_units(i).origin_position + ...
                    motion_units(i).l1 * e_delta_theta_phi * motion_units(i).lateral_ratio;
            motion_units(i).err_recorded = dt;
            if(dt<errp_cond && abs(dt - motion_units(i).err_recorded)<errp_cond)
                motion_units(i).err_flag = 0;
            end
            
        case 6 %   6 球铰运动单元    SMU
            theta = acos(- u1x'*u2x);
            theta_lim = motion_units(i).constraint_limit.theta_lim(2);
            if(theta>theta_lim)
                ee3 = cross(- u1x,u2x);
                ee4 = cross(u2x,ee3);ee4 = ee4 / norm(ee4);
                ee3 = ee4 * sin(theta_lim) + u2x * cos(theta_lim);
                u1x = - ee3/norm(ee3);
                
                motion_units(i).origin_position = motion_units(i).joint_position - ...
                    motion_units(i).l1 * u1x;
            else
                motion_units(i).origin_position = motion_units(i).joint_position - ...
                    motion_units(i).l1 * u1x;
            end

            tv = motion_units(i-1).de2;
            u1y = cross(tv,u1x);
            u1z = cross(u1x,u1y);
            u1z = u1z / norm(u1z);
            motion_units(i).origin_direction_position = motion_units(i).origin_position + ...
            motion_units(i).l2 * u1z * motion_units(i).lateral_ratio;

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




%% beifen 
% 0
%                 theta = motion_units(i).theta;
%                 if(theta<1e-6)
%                     u1x = -u2x;
%                     u1z = u2z*cos(phi)-u2y*sin(phi);
%                 else
%                     delta = phi - motion_units(i).delta;
%                     u2y = cross(u2z,u2x);
%                     u2y = u2y / norm(u2y);
%                     e_delta = u2z*cos(delta) - u2y*sin(delta);
%                     u1x = -(u2x*cos(theta) + e_delta * sin(theta));
%                     e_delta_theta = - sin(theta) * u2x + cos(theta) * e_delta;
%                     tem_y = cross(-u1x,e_delta_theta);
%                     e_delta_theta_delta = cos(-delta) * e_delta_theta + sin(-delta) * tem_y;
%                     u1y_delta_theta = cross(-u1x, e_delta_theta_delta);
%                     u1y_delta_theta = u1y_delta_theta / norm(u1y_delta_theta);
%                     u1z = cos(phi) * e_delta_theta_delta + sin(phi) * u1y_delta_theta;
%                 end
%                 u1x = u1x / norm(u1x);
%                 u1z = u1z / norm(u1z);
%                 motion_units(i).origin_position = motion_units(i).joint_position - ...
%                         u1x * motion_units(i).l1;
%                 motion_units(i).origin_direction_position = motion_units(i).origin_position + ...
%                     motion_units(i).l2 * u1z * motion_units(i).lateral_ratio;
