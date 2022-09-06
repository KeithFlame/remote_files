target=[-100;200];
%  target=pos(:,61225);
scatter(target(1),target(2),'filled');
axis equal 
axis([-150 150 -10 350])
hold on 

pc1=planar_continuum(0,100,[0;0],eye(2),[0.8,0,0]);
pc2=planar_continuum(0,100,pc1.endtip,eye(2),[0,0.8,0]);
pc3=planar_continuum(0.1,100,pc2.endtip,eye(2),[0,0,0.8]);
pc=[pc1,pc2,pc3];
pc(1)=pc(1).continuum_plot;
pc(2)=pc(2).continuum_plot;
pc(3)=pc(3).continuum_plot;


error=norm(target-pc(end).endtip);

iter=0;
frame_num=1;
%     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\ѧϰ\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
%     
while error(end)>=1e-2
    iter=iter+1;
    %%%% backward %%%%%%%%
    n=size(pc,2);
    
    
    l=line([target(1), pc(n).virtual_joint(1)],[target(2), pc(n).virtual_joint(2)],'LineStyle','--');
%     theta_dis=acos(dot(pc(n).virtual_joint-target,[-pc(n).rotation(1,2);-pc(n).rotation(2,2)])/norm(pc(n).virtual_joint-target))*sign(dot([pc(n).rotation(1,1);pc(n).rotation(2,1)],pc(n).virtual_joint-target));
    theta_dis=acos(dot(pc(n).virtual_joint-target,[-pc(n).rotation(1,2);-pc(n).rotation(2,2)])/norm(pc(n).virtual_joint-target));
    theta_dis=min(theta_dis,pi/2);
    theta_dis=theta_dis*sign(dot([pc(n).rotation(1,1);pc(n).rotation(2,1)],pc(n).virtual_joint-target));
    if theta_dis-(-pc(n).theta)<0.001 && abs(theta_dis)>0.001
%         rand('seed',0);
        theta_dis=theta_dis+random('Normal',0,0.01,1,1);
    end
    
    
    
    rotation_temp=[[cos(-pi/2),-sin(-pi/2);sin(-pi/2),cos(-pi/2)]*(pc(n).virtual_joint-target)/norm(pc(n).virtual_joint-target),(pc(n).virtual_joint-target)/norm(pc(n).virtual_joint-target)];
    
    pc(n).theta=theta_dis;
    pc(n).origin=target;
    pc(n).rotation=rotation_temp;
    pc(n)=pc(n).continuum_plot_update;
    set(l,'LineStyle','none');
    pause(0.1);
    
%     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\ѧϰ\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
    
    n=n-1;
    
    joint_temp=pc(n+1).endtip+pc(n+1).endrotation(:,2)*pc(n).tangent_length;
    l=line([pc(n+1).endtip(1), joint_temp(1)],[pc(n+1).endtip(2), joint_temp(2)],'LineStyle','--');
%     theta_dis=acos(dot(pc(n+1).endrotation(:,2),pc(n-1).virtual_joint-joint_temp)/norm(pc(n-1).virtual_joint-joint_temp))*sign(dot(pc(n-1).virtual_joint-joint_temp,pc(n+1).endrotation(:,1)));
    theta_dis=acos(dot(pc(n+1).endrotation(:,2),pc(n-1).virtual_joint-joint_temp)/norm(pc(n-1).virtual_joint-joint_temp));
    theta_dis=min(theta_dis,pi/2);
    theta_dis=theta_dis*sign(dot(pc(n-1).virtual_joint-joint_temp,pc(n+1).endrotation(:,1)));
    
    pc(n).theta=theta_dis;
    pc(n).origin=pc(n+1).endtip;
    pc(n).rotation=pc(n+1).endrotation;
    pc(n)=pc(n).continuum_plot_update;
    set(l,'LineStyle','none');
    pause(0.1);
%     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\ѧϰ\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
    
    n=n-1;
    
%     theta_dis=acos(dot(-pc(n+1).endrotation(:,2),pc(n).rotation(:,2)))*sign(dot(pc(n).rotation(:,1),-pc(n+1).endrotation(:,2)));
    theta_dis=acos(dot(-pc(n+1).endrotation(:,2),pc(n).rotation(:,2)));
    theta_dis=min(theta_dis,pi/2);
    theta_dis=theta_dis*sign(dot(pc(n).rotation(:,1),-pc(n+1).endrotation(:,2)));
    
    pc(n).theta=theta_dis;
    pc(n)=pc(n).continuum_plot_update;
    pause(0.1);
%     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\ѧϰ\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
    
    
    %%%% forward %%%%%%%%
    joint_temp=pc(n).endtip+pc(n).endrotation(:,2)*pc(n+1).tangent_length;
    l=line([pc(n).endtip(1), joint_temp(1)],[pc(n).endtip(2), joint_temp(2)],'LineStyle','--');
    
    n=n+1;
    
%     theta_dis=acos(dot(pc(n-1).endrotation(:,2),pc(n+1).virtual_joint-joint_temp)/norm(joint_temp-pc(n+1).virtual_joint))*sign(dot(pc(n+1).virtual_joint-joint_temp,pc(n-1).endrotation(:,1)));
    theta_dis=acos(dot(pc(n-1).endrotation(:,2),pc(n+1).virtual_joint-joint_temp)/norm(joint_temp-pc(n+1).virtual_joint));
    theta_dis=min(theta_dis,pi/2);
    theta_dis=theta_dis*sign(dot(pc(n+1).virtual_joint-joint_temp,pc(n-1).endrotation(:,1)));
    
    pc(n).theta=theta_dis;
    pc(n).origin=pc(n-1).endtip;
    pc(n).rotation=pc(n-1).endrotation;
	pc(n)=pc(n).continuum_plot_update;
    set(l,'LineStyle','none');
    pause(0.1);
%     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\ѧϰ\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
    
    
    n=n+1;
    
    joint_temp=pc(n-1).endtip+pc(n-1).endrotation(:,2)*pc(n).tangent_length;
    l=line([pc(n-1).endtip(1), joint_temp(1)],[pc(n-1).endtip(2), joint_temp(2)],'LineStyle','--');
%     theta_dis=acos(dot(pc(n-1).endrotation(:,2),target-joint_temp)/norm(target-joint_temp))*sign(dot(target-joint_temp,pc(n-1).endrotation(:,1)));
    theta_dis=acos(dot(pc(n-1).endrotation(:,2),target-joint_temp)/norm(target-joint_temp));
    theta_dis=min(theta_dis,pi/2);
    theta_dis=theta_dis*sign(dot(target-joint_temp,pc(n-1).endrotation(:,1)));
    
    pc(n).theta=theta_dis;
    pc(n).origin=pc(n-1).endtip;
    pc(n).rotation=pc(n-1).endrotation;
    pc(n)=pc(n).continuum_plot_update;
    set(l,'LineStyle','none');
    pause(0.1);
%     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\ѧϰ\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
    


    theta1(iter) = pc(1).theta;
    theta2(iter) = pc(2).theta;
    theta3(iter) = pc(3).theta;
    error(iter+1)=norm(target-pc(end).endtip);
    
end