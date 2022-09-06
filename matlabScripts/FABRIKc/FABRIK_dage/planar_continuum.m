classdef planar_continuum
    properties
        theta                   %
        length                  %
        origin                  %
        endtip
        rotation                %
        endrotation
        
        virtual_joint
        tangent_length
        
        plot_hd
        
        arc_color               %
    end
    methods
        
        function obj=planar_continuum(theta,length,origin,rotation,arc_color)
            if nargin<5
              obj.theta=0;
              obj.length=100;
              obj.origin=[0;0];
              obj.endtip=[0;obj.length];
              obj.rotation=[1,0;0,1];
              obj.endrotation=obj.rotation;   
              obj.arc_color=[0.5,0.5,0.5];
%               obj.virtual_joint=[0;obj.length/2];
%               obj.tangent_length=obj.length/2;
              obj=obj.variable_com;
              
            end
            
            
           if nargin==5
              obj.theta=theta;
              obj.length=length;
              obj.origin=origin;
              obj.rotation=rotation;
              obj.arc_color=arc_color;
              obj=obj.variable_com;
           end
        end
        
        function obj=variable_com(obj)
            if obj.theta==0
                obj.endtip=obj.origin+obj.rotation*[0;obj.length];
              else
                obj.endtip=obj.origin+obj.rotation*[obj.length/obj.theta*(1-cos(obj.theta));obj.length/obj.theta*sin(obj.theta)];
            end
              obj.endrotation=obj.rotation*[cos(-obj.theta), -sin(-obj.theta); sin(-obj.theta),cos(-obj.theta)];
              
              if obj.theta==0
                obj.tangent_length=obj.length/2;
                obj.virtual_joint=obj.origin+obj.rotation*[0;obj.tangent_length];
              else
                obj.tangent_length=obj.length/obj.theta*tan(obj.theta/2);
                obj.virtual_joint=obj.origin+obj.rotation*[0;obj.tangent_length];
              end
        end
        
        function obj=continuum_plot(obj)
            obj=obj.variable_com;
            base1=obj.origin+obj.rotation*[2;0];
            base2=obj.origin+obj.rotation*[2;0.5];
            base3=obj.origin+obj.rotation*[-2;0.5];
            base4=obj.origin+obj.rotation*[-2;0];
            X=[base1(1);base2(1);base3(1);base4(1)];
            Y=[base1(2);base2(2);base3(2);base4(2)];
            base_hd=fill(X,Y,'b');
%             hold on
%             axis equal
            
            end1=obj.endtip+obj.endrotation*[2;0];
            end2=obj.endtip+obj.endrotation*[2;-0.5];
            end3=obj.endtip+obj.endrotation*[-2;-0.5];
            end4=obj.endtip+obj.endrotation*[-2;0];
            X=[end1(1);end2(1);end3(1);end4(1)];
            Y=[end1(2);end2(2);end3(2);end4(2)];
            end_hd=fill(X,Y,'b');
            
            joint_hd=scatter(obj.virtual_joint(1),obj.virtual_joint(2),100,'MarkerEdgeColor',obj.arc_color,'LineWidth',1);
            tangent_hd=line([obj.origin(1),obj.virtual_joint(1),obj.endtip(1)],[obj.origin(2),obj.virtual_joint(2),obj.endtip(2)],'LineStyle','-.','Color',obj.arc_color,'LineWidth',2);
            
            t=linspace(0,1,30);
            if obj.theta==0
                temp=obj.origin+obj.rotation*[zeros(size(t,1),size(t,2));obj.length.*t];
                X=temp(1,:);
                Y=temp(2,:);
                arc_hd=line(X,Y,'Color',obj.arc_color,'LineWidth',2);
            else
                temp=obj.origin+obj.rotation*[obj.length/obj.theta*(1-cos(t.*obj.theta));obj.length/obj.theta*sin(t.*obj.theta)];
                X=temp(1,:);
                Y=temp(2,:);
                arc_hd=line(X,Y,'Color',obj.arc_color,'LineWidth',2);
            end
            obj.plot_hd=[base_hd,end_hd,joint_hd,tangent_hd,arc_hd];
            
%             [X,Y]=obj.origin+obj.rotation*[obj.length/obj.theta*(1-cos(obj.theta));obj.length/obj.theta*sin(obj.theta)];
        end
        
        function obj=continuum_plot_update(obj)
            obj=obj.variable_com;

            base1=obj.origin+obj.rotation*[2;0];
            base2=obj.origin+obj.rotation*[2;0.5];
            base3=obj.origin+obj.rotation*[-2;0.5];
            base4=obj.origin+obj.rotation*[-2;0];
            X=[base1(1);base2(1);base3(1);base4(1)];
            Y=[base1(2);base2(2);base3(2);base4(2)];
%             base_hd=fill(X,Y,'b');
            set(obj.plot_hd(1),'XData',X,'YData',Y);
            
            end1=obj.endtip+obj.endrotation*[2;0];
            end2=obj.endtip+obj.endrotation*[2;-0.5];
            end3=obj.endtip+obj.endrotation*[-2;-0.5];
            end4=obj.endtip+obj.endrotation*[-2;0];
            X=[end1(1);end2(1);end3(1);end4(1)];
            Y=[end1(2);end2(2);end3(2);end4(2)];
%             end_hd=fill(X,Y,'b');
            set(obj.plot_hd(2),'XData',X,'YData',Y);
            
%             joint_hd=scatter(obj.virtual_joint(1),obj.virtual_joint(2));
            set(obj.plot_hd(3),'XData',obj.virtual_joint(1),'YData',obj.virtual_joint(2));
%             tangent_hd=line([obj.origin(1),obj.virtual_joint(1),obj.endtip(1)],[obj.origin(2),obj.virtual_joint(2),obj.endtip(2)],'LineStyle','-.');
            set(obj.plot_hd(4),'XData',[obj.origin(1),obj.virtual_joint(1),obj.endtip(1)],'YData',[obj.origin(2),obj.virtual_joint(2),obj.endtip(2)]);
            
            t=linspace(0,1,30);
            if obj.theta==0
                temp=obj.origin+obj.rotation*[zeros(size(t,1),size(t,2));obj.length.*t];
                X=temp(1,:);
                Y=temp(2,:);
%                 arc_hd=line(X,Y);
            else
                temp=obj.origin+obj.rotation*[obj.length/obj.theta*(1-cos(t.*obj.theta));obj.length/obj.theta*sin(t.*obj.theta)];
                X=temp(1,:);
                Y=temp(2,:);
%                 arc_hd=line(X,Y);
            end
%             obj.plot_hd=[base_hd,end_hd,joint_hd,tangent_hd,arc_hd];
            set(obj.plot_hd(5),'XData',X,'YData',Y);
        end
    end
end
            
            