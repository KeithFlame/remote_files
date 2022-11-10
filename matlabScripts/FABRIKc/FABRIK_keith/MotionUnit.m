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
        end_position
        origin_position
        joint_position
        
        
        l1
        l2
        l0
        theta
        delta
        e1
        e2
        is_plot
        joint_type
        color
        hd

        constraint_limit
    end
    
    methods
        function obj = MotionUnit(origin_position,joint_position,...
                end_position,joint_type,color,constraint_limit)
            %MOTIONUNIT 构造此类的实例
            %   此处显示详细说明
            if(nargin == 3)
                joint_type = 2;
                color = rand(1,3);
                constraint_limit.l1_lim = [1e-6 150];
                constraint_limit.l2_lim = [1e-6 150];
                constraint_limit.theta_lim = [0 pi/3*2];
            end
            if(nargin == 4)
                color = rand(1,3);
                constraint_limit.l1_lim = [1e-6 150];
                constraint_limit.l2_lim = [1e-6 150];
                constraint_limit.theta_lim = [0 pi/3*2];
            end
            if(nargin == 5)
                constraint_limit.l1_lim = [1e-6 150];
                constraint_limit.l2_lim = [1e-6 150];
                constraint_limit.theta_lim = [0 pi/3*2];
            end
            obj.color = color;
            obj.end_position = reshape(end_position,[3 1]);
            obj.joint_position = reshape(joint_position,[3 1]);
            obj.origin_position = reshape(origin_position,[3 1]);
            obj.is_plot = 0;
            obj.joint_type = joint_type;
            obj.constraint_limit = constraint_limit;
            obj.l0 = 0;
            obj = obj.refreshUnit;
        end
        
        function obj = refreshUnit(obj)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            [m,n] = size([obj.origin_position obj.joint_position obj.end_position]);

            assert((m == 3) && (n == 3));
            obj.l1 = norm(obj.origin_position - obj.joint_position);
            obj.l2 = norm(obj.end_position - obj.joint_position);
            obj.e1 = (obj.origin_position - obj.joint_position)/obj.l1;
            obj.e2 = (obj.end_position - obj.joint_position)/obj.l2;
            obj.theta = acos(dot(-obj.e1,obj.e2));
            obj.delta = acos(dot(-obj.e1(1:2)/norm(obj.e1(1:2)),...
                obj.e2(1:2)/norm(obj.e2(1:2)))); 

        end
        
        function obj = plotUnit(obj)
            obj = obj.refreshUnit;
            if(obj.is_plot)
                color1 = obj.color;
                color2 = 1-color1;
                P = [obj.origin_position obj.joint_position obj.end_position];
                if(obj.joint_type == 0)
                    pj1 = plot3(P(1,2),P(2,2),P(3,2));
                else
                    pj1 = plot3(P(1,2),P(2,2),P(3,2),'Color',color2,'Marker','o');
                end
                [x,y,z] = obj.plotCircle_keith(obj.e1,obj.origin_position);
                pj2 = plot3(x,y,z,'Color',color2);
                [x,y,z] = obj.plotCircle_keith(obj.e2,obj.end_position);
                pj3 = plot3(x,y,z,'Color',color2);
                if(obj.joint_type == 3 && obj.theta >1e-6)
                    pl = plot3(P(1,:),P(2,:),P(3,:),'Color',color1,LineStyle='-.');
                    pl.Color(4) = 0.7;
                    p1 = ((P(:,1)+P(:,3))/2 - P(:,2))/norm((P(:,1)+P(:,3))/2 - P(:,2));
                    c = P(:,2) + p1 * obj.l1/sin(obj.theta/2);
                    r = obj.l1/tan(obj.theta/2);
                    [x,y,z] = obj.pot2plot3(P(:,1),P(:,3),c,r);
                    pa = plot3(x,y,z,'Color',color2,LineWidth=3);
                    pa.Color(4) = 0.7;
                    
                else
                    pl = plot3(P(1,:),P(2,:),P(3,:),'Color',color1,LineWidth=3);
                    pl.Color(4) = 0.7;
                    c = P(:,2);
                    r = 1;
                    [x,y,z] = obj.pot2plot3(P(:,1),P(:,3),c,r);
                    pa = plot3(x,y,z,'Color',color1,LineWidth=3);
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
                
                [x,y,z] = obj.plotCircle_keith(obj.e1,obj.origin_position);
                set(obj.hd(3),'XData',x,'YData',y,'ZData',z);
                [x,y,z] = obj.plotCircle_keith(obj.e2,obj.end_position);
                set(obj.hd(4),'XData',x,'YData',y,'ZData',z);
                if(obj.joint_type == 3)
                    set(obj.hd(1),'LineStyle','none');
                    if(abs(obj.theta) < 1e-7)
                        obj.theta = 1e-7;
                        n = P(:,1);
                        m = P(:,3)-[x(1) y(1) z(1)]';
                        
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
        
        function [x0,y0,z0] = plotCircle_keith(obj,n0,p0)
            %
            %
            %
            %
            n=n0;
            r = 3;
            theta0=(0:2*pi/100:2*pi)'; %theta角从0到2*pi
            a=cross(n,[1 0 0]);       %n与i叉乘，求取a向量
            if ~any(a) %如果a为零向量，将n与j叉乘   %any检测矩阵中是否有非零元素，如果有，则返回1，否则，返回0
                a=cross(n,[0 1 0]);
            end
            b=cross(n,a); %求取b向量
            a=a/norm(a);  %单位化a向量
            b=b/norm(b);  %单位化b向量
            %a,b是满足既垂直于n，又互相垂直的任意单位向量
            
            %底面圆方程
            c1=p0(1)*ones(size(theta0,1),1);
            c2=p0(2)*ones(size(theta0,1),1);
            c3=p0(3)*ones(size(theta0,1),1);
            x0=c1+r*a(1)*cos(theta0)+r*b(1)*sin(theta0); %圆上各点的x坐标
            y0=c2+r*a(2)*cos(theta0)+r*b(2)*sin(theta0); %圆上各点的y坐标
            z0=c3+r*a(3)*cos(theta0)+r*b(3)*sin(theta0); %圆上各点的z坐标
        end
    
    end
end

