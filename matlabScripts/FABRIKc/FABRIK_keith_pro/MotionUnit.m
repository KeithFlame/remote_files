classdef MotionUnit
    %MOTIONUNIT 此处显示有关此类的摘要
    %   此处显示详细说明
    %   声明，不同的joint_type对应不同的关节
    %   0 刚体运动单元    FMU    -
    %   1 移动运动单元    PMU    l1
    %   2 万向节运动单元   UMU    theta, delta
    %   3 连续体运动单元   CMU    l1, l2, theta, delta
    %   4 轴向旋转运动单元    PiMU    theta
    %   5 径向旋转运动单元  RMU    delta
    %   6 球铰运动单元    SMU    theta, delta
    %

    properties
        end_position                    % 末端位置
        origin_position                 % 起始位置
        joint_position                  % 旋转中心位置
        end_direction_position          % 末端Z方向的终止位置
        origin_direction_position       % 起始Z方向的终止位置
        lateral_ratio                   % 末端和起始的Z方向长度
        l1                              % l1长度值
        l2                              % l2长度值
        err_recorded                    % 当前误差
        err_flag                        % 修正flag
        theta                           % 第一段与第二段之间的夹角
        delta                           % 第一段与第二段的平面角 -u2x的投影与u1z之间的夹角。（从u1z转到-u2x）
        phi                             % 起始与末端的夹角
        e1                              % 第一段的方向，由旋转中心位置指向起始位置
        e2                              % 第二段的方向，由旋转中心位置指向末端位置
        de1                             % 起始Z方向
        de2                             % 末端Z方向
        is_plot                         % 是否画图标志位，默认不画图
        joint_type                      % 运动单元类型标志位，默认为万向节运动单元
        color                           % 运动单元的颜色，默认颜色随机
        hd                              % 画图的句柄
        constraint_limit                % 运动单元的约束
    end
    
    methods
        function obj = MotionUnit(origin_position, joint_position, ...
                end_position, origin_direction_position, end_direction_position, ...
                joint_type,color,constraint_limit)
            %MOTIONUNIT 构造此类的实例
            %   此处显示详细说明
            if(nargin == 5)
                joint_type = 2;
                color = rand(1,3);
                constraint_limit.l1_lim = [1e-6 150];
                constraint_limit.l2_lim = [1e-6 150];
                constraint_limit.theta_lim = [0 pi/2*2];
            end
            if(nargin == 6)
                color = rand(1,3);
                constraint_limit.l1_lim = [1e-6 150];
                constraint_limit.l2_lim = [1e-6 150];
                constraint_limit.theta_lim = [0 pi/2*2];
            end
            if(nargin == 7)
                constraint_limit.l1_lim = [1e-6 150];
                constraint_limit.l2_lim = [1e-6 150];
                constraint_limit.theta_lim = [0 pi/2*2];
            end
            obj.color = color;
            obj.end_position = reshape(end_position,[3 1]);
            obj.joint_position = reshape(joint_position,[3 1]);
            obj.origin_position = reshape(origin_position,[3 1]);
            obj.end_direction_position = reshape(end_direction_position,[3 1]);
            obj.origin_direction_position = reshape(origin_direction_position,[3 1]);
            obj.is_plot = 0;
            obj.joint_type = joint_type;
            obj.constraint_limit = constraint_limit;
            obj.err_recorded = 10;
            obj.err_flag = 0;
            obj.lateral_ratio = 0.4;
            
            if(joint_type ~= 3 )     % 纯粹减少计算量
                obj.l1 = norm(obj.origin_position - obj.joint_position);
                obj.l2 = norm(obj.end_position - obj.joint_position);
            end
            obj = obj.refreshUnit;

            
            if(joint_type == 0)
                or_p = obj.origin_direction_position;
                en_p = obj.end_direction_position;
    
                tem_ee1 = or_p - obj.origin_position;
                tem_ee1 = tem_ee1/norm(tem_ee1);
                try
                    assert(tem_ee1'*obj.e1 < 1e-6);
                catch
                    warning(['Problem occurred by dot_between_ORIGIN_direction_and_link.' ...
                        ' Assert the value of non-zero but %f\n.'],tem_ee1'*obj.e1);
                    eet = cross(tem_ee1,obj.e1);
                    tem_ee1 = cross(obj.e1, eet);
                end        
                
                tem_ee2 = en_p - obj.end_position;
                tem_ee2 = tem_ee2 / norm(tem_ee2);
                try
                    assert(tem_ee2'*obj.e2 < 1e-6);
                catch
                    warning(['Problem occurred by dot_between_END_direction_and_link.' ...
                        ' Assert the value of non-zero but %f\n.'],tem_ee2'*obj.e2);
                    eet = cross(tem_ee2,obj.e2);
                    tem_ee2 = cross(obj.e2, eet);
                end
                            
                obj.de1 = tem_ee1;
                obj.de1 = obj.de1 / norm(obj.de1);
                obj.origin_direction_position = obj.origin_position + ...
                        obj.de1 * obj.l2 *obj.lateral_ratio;
                obj.de2 = tem_ee2;
                obj.de2 = obj.de2 / norm(obj.de2);
                obj.end_direction_position = obj.end_position + ...
                        obj.de2 * obj.l2 *obj.lateral_ratio;
                dot_de12 = obj.setDotValue(obj.de1,obj.de2);
                obj.phi = acos(dot_de12);

                obj.e1 = (obj.origin_position - obj.joint_position)/norm(obj.origin_position - obj.joint_position);
                obj.e2 = (obj.end_position - obj.joint_position)/norm(obj.end_position - obj.joint_position);
                dot_e12 = obj.setDotValue(-obj.e1,obj.e2);
                obj.theta = acos(dot_e12);
                
                dot_mu2x_u1y = obj.setDotValue(obj.e2, cross(obj.de1,-obj.e1));
                dot_mu2x_u1z = obj.setDotValue(obj.e2, obj.de1);
                if(abs(dot_mu2x_u1z) < 1e-6 && abs(dot_mu2x_u1y) < 1e-6)
                    obj.delta = 0;                
                else
                    obj.delta = atan2(-dot_mu2x_u1y,dot_mu2x_u1z);
                end
            else
                dot_de12 = obj.setDotValue(obj.de1,obj.de2);
                obj.phi = acos(dot_de12);
            end
            

        end
        
        function obj = refreshUnit(obj)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            if(obj.joint_type == 3 || obj.joint_type == 1)
                obj.l1 = norm(obj.origin_position - obj.joint_position);
                obj.l2 = norm(obj.end_position - obj.joint_position);
            else
                a1 = abs(norm(obj.origin_position - obj.joint_position)-obj.l1) < 1e-6;
                a2 = abs(norm(obj.end_position - obj.joint_position)-obj.l2) < 1e-6;
                try
                    assert( a1 && a2);
                catch
                    warning(['Problem occurred by link_length.' ...
                        ' Assert the value of non-equal but %f and %f\n.'],a1,a2);

                end 

            end
            obj.e1 = (obj.origin_position - obj.joint_position)/norm(obj.origin_position - obj.joint_position);
            obj.e2 = (obj.end_position - obj.joint_position)/norm(obj.end_position - obj.joint_position);
            dot_e12 = obj.setDotValue(-obj.e1,obj.e2);
            

            or_p = obj.origin_direction_position;
            en_p = obj.end_direction_position;

            tem_ee1 = or_p - obj.origin_position;
            tem_ee1 = tem_ee1/norm(tem_ee1);
            try
                assert(tem_ee1'*obj.e1 < 1e-6);
            catch
                warning(['Problem occurred by dot_between_ORIGIN_direction_and_link.' ...
                    ' Assert the value of non-zero but %f\n.'],tem_ee1'*obj.e1);
                eet = cross(tem_ee1,obj.e1);
                tem_ee1 = cross(obj.e1, eet);
            end        
            
            tem_ee2 = en_p - obj.end_position;
            tem_ee2 = tem_ee2 / norm(tem_ee2);
            try
                assert(tem_ee2'*obj.e2 < 1e-6);
            catch
                warning(['Problem occurred by dot_between_END_direction_and_link.' ...
                    ' Assert the value of non-zero but %f\n.'],tem_ee2'*obj.e2);
                eet = cross(tem_ee2,obj.e2);
                tem_ee2 = cross(obj.e2, eet);
            end
                        
            obj.de1 = tem_ee1;
            obj.de1 = obj.de1 / norm(obj.de1);
            obj.origin_direction_position = obj.origin_position + ...
                    obj.de1 * obj.l2 *obj.lateral_ratio;
            obj.de2 = tem_ee2;
            obj.de2 = obj.de2 / norm(obj.de2);
            obj.end_direction_position = obj.end_position + ...
                    obj.de2 * obj.l2 *obj.lateral_ratio;

            
            if(obj.joint_type == 4)
                dot_mu2x_u1y = obj.setDotValue(obj.de2, cross(obj.de1,-obj.e1));
                dot_mu2x_u1z = obj.setDotValue(obj.de2, obj.de1);
                obj.delta = atan2(-dot_mu2x_u1y,dot_mu2x_u1z);
                return;
            end
            if(obj.joint_type == 0)

            else
                obj.theta = acos(dot_e12);
    
                dot_mu2x_u1y = obj.setDotValue(obj.e2, cross(obj.de1,-obj.e1));
                dot_mu2x_u1z = obj.setDotValue(obj.e2, obj.de1);
                if(abs(dot_mu2x_u1z) < 1e-6 && abs(dot_mu2x_u1y) < 1e-6)
                    obj.delta = 0;                
                else
                    obj.delta = atan2(-dot_mu2x_u1y,dot_mu2x_u1z);
                end
            end
            
        end
        
        function obj = plotUnit(obj)
            obj = obj.refreshUnit;
            if(obj.is_plot)
                color1 = obj.color;
                color2 = color1;
                P = [obj.origin_position obj.joint_position obj.end_position];
                if(obj.joint_type == 0)
                    pj1 = plot3(P(1,2),P(2,2),P(3,2));
                else
                    pj1 = plot3(P(1,2),P(2,2),P(3,2),'Color',color2,'Marker','o','MarkerSize',...
                        obj.lateral_ratio*obj.l2/2);
                end

                P_lateral_origin = [obj.origin_position obj.origin_direction_position];
                pj2 = plot3(P_lateral_origin(1,:), ...
                    P_lateral_origin(2,:), P_lateral_origin(3,:), 'Color', color2);
                P_lateral_origin = [obj.end_position obj.end_direction_position];
                pj3 = plot3(P_lateral_origin(1,:), ...
                    P_lateral_origin(2,:), P_lateral_origin(3,:), 'Color', color2);
                if(obj.joint_type == 3 && obj.theta >1e-6)
                    pl = plot3(P(1,:),P(2,:),P(3,:),'Color',color1,LineStyle='-.');
                    pl.Color(4) = 0.7;
                    p1 = ((P(:,1)+P(:,3))/2 - P(:,2))/norm((P(:,1)+P(:,3))/2 - P(:,2));
                    c = P(:,2) + p1 * obj.l1/sin(obj.theta/2);
                    r = obj.l1/tan(obj.theta/2);
                    [x,y,z] = obj.pot2plot3(P(:,1),P(:,3),c,r);
                    pa = plot3(x,y,z,'Color',color2,LineWidth=5);
                    pa.Color(4) = 0.7;
                    
                else
                    pl = plot3(P(1,:),P(2,:),P(3,:),'Color',color1,LineWidth=5);
                    pl.Color(4) = 0.7;
                    c = P(:,2);
                    r = 1;
                    [x,y,z] = obj.pot2plot3(P(:,1),P(:,3),c,r);
                    pa = plot3(x,y,z,'Color',color1,LineWidth=5);
                end

                obj.hd = [pl, pj1, pj2, pj3, pa];
            end
        end
        
        function obj = updateUnit(obj)
            obj = obj.refreshUnit;
            if(obj.is_plot)
                P = [obj.origin_position obj.joint_position obj.end_position];
                if(obj.joint_type == 0)
                    set(obj.hd(2),'LineStyle','none');
                else
                    set(obj.hd(2),'XData',P(1,2),'YData',P(2,2),'ZData',P(3,2));
                end
                P_lateral_origin = [obj.origin_position obj.origin_direction_position];
                set(obj.hd(3), 'XData', P_lateral_origin(1,:), ...
                    'YData', P_lateral_origin(2,:), 'ZData',P_lateral_origin(3,:));
                P_lateral_origin = [obj.end_position obj.end_direction_position];
                set(obj.hd(4), 'XData', P_lateral_origin(1,:), ...
                    'YData', P_lateral_origin(2,:), 'ZData',P_lateral_origin(3,:));
                if(obj.joint_type == 3)
                    set(obj.hd(1),'LineStyle','none');
                    if(abs(obj.theta) < 1e-7)
                        obj.theta = 1e-7;
                        n = P(:,1);
                        m = P(:,3)-obj.end_direction_position;
                        
                        p1 = [m(3)*n(2)-m(2)*n(3) n(3)*m(1)-n(1)*m(3) n(1)*m(2)-m(1)*n(2)]';
                        p1 = p1/norm(p1);
                    else                    
                        p1 = ((P(:,1)+P(:,3))/2 - P(:,2))/norm((P(:,1)+P(:,3))/2 - P(:,2));                       
                    end
                    c = P(:,2) + p1 * obj.l1/sin(obj.theta/2);
                    r = obj.l1/tan(obj.theta/2);
                    [x,y,z] = obj.pot2plot3(P(:,1),P(:,3),c,r);
                    set(obj.hd(5),'XData',x','YData',y','ZData',z');
                else
                    set(obj.hd(1),'XData',P(1,:),'YData',P(2,:),'ZData',P(3,:));
                    set(obj.hd(5),'LineStyle','none');
                end
            end
        end
        
        function [x,y,z] = pot2plot3(obj,p1,p2,c,r)
            % 画空间圆弧
            %   输入弧起点p1,终点p2,圆心c
            %% 平面 法向量  
            p12 = p2 - p1;
            pc1 = p1 - c;
            pc2 = p2 - c;
            pc1 = pc1/norm(pc1);
            pc2 = pc2/norm(pc2);
            n = cross(p12,pc2);%法向量
            n = n/norm(n);
            u = pc1/norm(pc1);
            v = cross(u,n);%以u为起始向量，满足右手定则坐标系，则夹角thetam是从u向v的
            v = v/norm(v);
            % v = -1.*v/norm(v); 
            dot_a = dot(pc1,pc2)/norm(pc1)/norm(pc2);
            if(abs(dot_a)>1)
                dot_a = sign(dot_a);
            end

            thetam = acos(dot_a);
            theta1 = (0:thetam/100:thetam)';
            c1=c(1)*ones(size(theta1,1),1);
            c2=c(2)*ones(size(theta1,1),1);
            c3=c(3)*ones(size(theta1,1),1);
            x=c1+r*u(1)*cos(theta1)+r*v(1)*sin(theta1);%圆上各点的x坐标
            y=c2+r*u(2)*cos(theta1)+r*v(2)*sin(theta1);%圆上各点的y坐标
            z=c3+r*u(3)*cos(theta1)+r*v(3)*sin(theta1);%圆上各点的z坐标
        end
           
        function cosine_value = setDotValue(obj,a,b)
            a = a / norm(a);
            b = b / norm(b);
            cosine_value = a'*b;
            if(cosine_value > 1)
                cosine_value = 1;
            elseif(cosine_value < -1)
                cosine_value = -1;
            end

        end

    end
end

