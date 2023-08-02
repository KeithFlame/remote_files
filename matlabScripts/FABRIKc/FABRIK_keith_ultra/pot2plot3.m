function [x,y,z] = pot2plot3(p1,p2,p3,theta_l)
            % 画空间圆弧
            %   输入弧起点p1,终点p2,圆心c
            if(theta_l<1e-6)
                x=linspace(p1(1),p3(1),101);
                y=linspace(p1(2),p3(2),101);
                z=linspace(p1(3),p3(3),101);
                return;
            end
            %% 平面 法向量  

            vec2 = (p1-p2)+(p3-p2);vec2 = vec2/norm(vec2);
            tem_l1 = norm(p1-p3)/2;
            tem_l2 = tem_l1/sin(theta_l/2);
            tem_l3 = tem_l2/cos(theta_l/2);
            c = p2+tem_l3*vec2;
            r = tem_l2;

            p12 = p3 - p1;
            pc1 = p1 - c;
            pc2 = p3 - c;
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