clear

target=[-100;200];
target_theta=-pi/2;
% target=[-0;207];
% target_theta=-0;
%  target=pos(:,61225);
scatter(target(1),target(2),'filled');
hold on 
scatter(0,0,'filled');
line([target(1) target(1)+20*cos(-target_theta+pi/2)],[target(2) target(2)+20*sin(-target_theta+pi/2)],'LineWidth',2,'Color',[0.5,0.5,0.5]);


set(gca,'XColor',[1 1 1],'YColor',[1 1 1]);

% width=1920;%宽度，像素数
% height=1080;%高度
% left=00;%距屏幕左下角水平距离
% bottem=00;%距屏幕左下角垂直距离
% set(gcf,'position',[left,bottem,width,height])
axis equal 
axis manual
axis([-150 150 -40 350])

pc1=planar_continuum(0.001,100,[0;0],eye(2),[0.8,0,0]);
pc2=planar_continuum(0,100,pc1.endtip,eye(2),[0,0.8,0]);
pc3=planar_continuum(0,100,pc2.endtip,eye(2),[0,0,0.8]);
pc=[pc1,pc2,pc3];
pc(1)=pc(1).continuum_plot;
pc(2)=pc(2).continuum_plot;
pc(3)=pc(3).continuum_plot;
pause(0.1);

error=norm(target-pc(end).endtip);
% ang_err=norm(target-pc(end).endtip);

iter=0;
frame_num=1;
%     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\学习\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
%     
while error(end)>=1e-2
    iter=iter+1;
    %%%% backward %%%%%%%%
    n=size(pc,2);
    
    rotation_temp=[[cos(-pi/2),-sin(-pi/2);sin(-pi/2),cos(-pi/2)]*[-cos(-target_theta+pi/2);-sin(-target_theta+pi/2)],[-cos(-target_theta+pi/2);-sin(-target_theta+pi/2)]];
    
    joint_temp=(target-pc(n).tangent_length*[cos(-target_theta+pi/2);sin(-target_theta+pi/2)]);
    joint=scatter(joint_temp(1),joint_temp(2),'MarkerEdgeColor',[.5,.5,0]);
    l1=line([target(1) joint_temp(1)],[target(2) joint_temp(2)],'LineStyle','--');
    pause(0.1);
    %     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\学习\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
%     
    
    l2=line([joint_temp(1), pc(n-1).virtual_joint(1)],[joint_temp(2), pc(n-1).virtual_joint(2)],'LineStyle','--');
    pause(0.1);
    %     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\学习\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
%     
    
    theta_dis=acos(dot([-cos(-target_theta+pi/2);-sin(-target_theta+pi/2)],pc(n-1).virtual_joint-joint_temp)/norm(pc(n-1).virtual_joint-joint_temp));
%     theta_dis=min(theta_dis,pi/2);
    theta_dis=theta_dis*sign(dot(rotation_temp(:,1),pc(n-1).virtual_joint-joint_temp));
%     theta_dis=acos(dot([-cos(-target_theta+pi/2);-sin(-target_theta+pi/2)],[-pc(n).rotation(1,2);-pc(n).rotation(2,2)]));
%     theta_dis=min(theta_dis,pi/2);
%     theta_dis=theta_dis*sign(dot([pc(n).rotation(1,1);pc(n).rotation(2,1)],[-cos(-target_theta+pi/2);-sin(-target_theta+pi/2)]));
%     if theta_dis-(-pc(n).theta)<0.0001 && abs(theta_dis)>0.0001
% %         rand('seed',0);
% %         theta_dis=theta_dis+random('Normal',0,0.01,1,1);
% % theta_dis=theta_dis+random('Uniform',-0.0001,0.0001,1,1);
%     pc(n).theta=theta_dis;
%     pc(n).origin=target;
%     pc(n).rotation=rotation_temp;
%     pc(n)=pc(n).variable_com;
%     theta_dis_temp=acos(dot([pc(n).rotation(1,2);pc(n).rotation(2,2)],pc(n-1).virtual_joint-pc(n).virtual_joint)/norm(pc(n-1).virtual_joint-pc(n).virtual_joint));
%     theta_dis_temp=theta_dis_temp*sign(dot([pc(n).rotation(1,1);pc(n).rotation(2,1)],pc(n-1).virtual_joint-pc(n).virtual_joint));
%     theta_dis=(theta_dis+theta_dis_temp)/2;
%     end

%     rotation_temp=[[cos(-pi/2),-sin(-pi/2);sin(-pi/2),cos(-pi/2)]*(pc(n).virtual_joint-target)/norm(pc(n).virtual_joint-target),(pc(n).virtual_joint-target)/norm(pc(n).virtual_joint-target)];
    

    pc(n).theta=theta_dis;
    pc(n).origin=target;
    pc(n).rotation=rotation_temp;
    pc(n)=pc(n).continuum_plot_update;
    pause(0.1);
    %     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\学习\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
%     
    
    
    set(l1,'LineStyle','none');
    set(l2,'LineStyle','none');
    set(joint,'Marker','none');
    pause(0.1);
    
%     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\学习\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;

    n=n-1;
    
    joint_temp=pc(n+1).endtip+pc(n+1).endrotation(:,2)*pc(n).tangent_length;
    joint=scatter(joint_temp(1),joint_temp(2),'MarkerEdgeColor',[.5,.5,0]);
    l1=line([pc(n+1).endtip(1), joint_temp(1)],[pc(n+1).endtip(2), joint_temp(2)],'LineStyle','--');
    pause(0.1);
    %     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\学习\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
%     
    l2=line([joint_temp(1), pc(n-1).virtual_joint(1)],[joint_temp(2), pc(n-1).virtual_joint(2)],'LineStyle','--');
    pause(0.1);
    %     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\学习\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
%     
    
%     theta_dis=acos(dot(pc(n+1).endrotation(:,2),pc(n-1).virtual_joint-joint_temp)/norm(pc(n-1).virtual_joint-joint_temp))*sign(dot(pc(n-1).virtual_joint-joint_temp,pc(n+1).endrotation(:,1)));
    theta_dis=acos(dot(pc(n+1).endrotation(:,2),pc(n-1).virtual_joint-joint_temp)/norm(pc(n-1).virtual_joint-joint_temp));
%     theta_dis=min(theta_dis,pi/2);
    theta_dis=theta_dis*sign(dot(pc(n-1).virtual_joint-joint_temp,pc(n+1).endrotation(:,1)));

%     if theta_dis-(-pc(n).theta)<0.0001 && abs(theta_dis)>0.0001
% %         rand('seed',0);
% %         theta_dis=theta_dis+random('Normal',0,0.01,1,1);
% theta_dis=theta_dis+random('Uniform',-0.0001,0.0001,1,1);
%     end
%     if theta_dis-(-pc(n).theta)<0.0001 && abs(theta_dis)>0.0001
% %         rand('seed',0);
% %         theta_dis=theta_dis+random('Normal',0,0.01,1,1);
% % theta_dis=theta_dis+random('Uniform',-0.0001,0.0001,1,1);
%     pc(n).theta=theta_dis;
%     pc(n).origin=target;
%     pc(n).rotation=rotation_temp;
%     pc(n)=pc(n).variable_com;
%     theta_dis_temp=acos(dot([pc(n).rotation(1,2);pc(n).rotation(2,2)],pc(n-1).virtual_joint-pc(n).virtual_joint)/norm(pc(n-1).virtual_joint-pc(n).virtual_joint));
%     theta_dis_temp=theta_dis_temp*sign(dot([pc(n).rotation(1,1);pc(n).rotation(2,1)],pc(n-1).virtual_joint-pc(n).virtual_joint));
%     theta_dis=(theta_dis+theta_dis_temp)/2;
%     end
    
    pc(n).theta=theta_dis;
    pc(n).origin=pc(n+1).endtip;
    pc(n).rotation=pc(n+1).endrotation;
    pc(n)=pc(n).continuum_plot_update;
    pause(0.1);
    %     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\学习\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
%     
    
    set(l1,'LineStyle','none');
    set(l2,'LineStyle','none');
    set(joint,'Marker','none');
    pause(0.1);
%     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\学习\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
    
    n=n-1;
    
%     theta_dis=acos(dot(-pc(n+1).endrotation(:,2),pc(n).rotation(:,2)))*sign(dot(pc(n).rotation(:,1),-pc(n+1).endrotation(:,2)));
    theta_dis=acos(dot(-pc(n+1).endrotation(:,2),pc(n).rotation(:,2)));
%     theta_dis=min(theta_dis,pi/2);
    theta_dis=theta_dis*sign(dot(pc(n).rotation(:,1),-pc(n+1).endrotation(:,2)));
    
%     if theta_dis-(-pc(n).theta)<0.0001 && abs(theta_dis)>0.0001
% %         rand('seed',0);
% %         theta_dis=theta_dis+random('Normal',0,0.01,1,1);
% theta_dis=theta_dis+random('Uniform',-0.0001,0.0001,1,1);
%     end
%     if theta_dis-(-pc(n).theta)<0.0001 && abs(theta_dis)>0.0001
% %         rand('seed',0);
% %         theta_dis=theta_dis+random('Normal',0,0.01,1,1);
% % theta_dis=theta_dis+random('Uniform',-0.0001,0.0001,1,1);
%     pc(n).theta=theta_dis;
%     pc(n).origin=target;
%     pc(n).rotation=rotation_temp;
%     pc(n)=pc(n).variable_com;
%     theta_dis_temp=acos(dot([pc(n).rotation(1,2);pc(n).rotation(2,2)],pc(n-1).virtual_joint-pc(n).virtual_joint)/norm(pc(n-1).virtual_joint-pc(n).virtual_joint));
%     theta_dis_temp=theta_dis_temp*sign(dot([pc(n).rotation(1,1);pc(n).rotation(2,1)],pc(n-1).virtual_joint-pc(n).virtual_joint));
%     theta_dis=(theta_dis+theta_dis_temp)/2;
%     end    
    pc(n).theta=-theta_dis;
    pc(n).origin=pc(n+1).endtip;
    pc(n).rotation=pc(n+1).endrotation;
    pc(n)=pc(n).continuum_plot_update;
    pause(0.1);
%     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\学习\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
    pc(n).theta=theta_dis;
    pc(n).origin=[0;0];
    pc(n).rotation=[1,0;0,1];
    pc(n)=pc(n).continuum_plot_update;
    pause(0.1);
    
    %%%% forward %%%%%%%%
    n=n+1;
    
    joint_temp=pc(n-1).endtip+pc(n-1).endrotation(:,2)*pc(n).tangent_length;
    joint=scatter(joint_temp(1),joint_temp(2),'MarkerEdgeColor',[.5,.5,0]);
    l1=line([pc(n-1).endtip(1), joint_temp(1)],[pc(n-1).endtip(2), joint_temp(2)],'LineStyle','--');
    pause(0.1);
    %     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\学习\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
%     
    l2=line([joint_temp(1), pc(n+1).virtual_joint(1)],[joint_temp(2), pc(n+1).virtual_joint(2)],'LineStyle','--');
    pause(0.1);
    %     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\学习\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
%     

%     theta_dis=acos(dot(pc(n-1).endrotation(:,2),pc(n+1).virtual_joint-joint_temp)/norm(joint_temp-pc(n+1).virtual_joint))*sign(dot(pc(n+1).virtual_joint-joint_temp,pc(n-1).endrotation(:,1)));
    theta_dis=acos(dot(pc(n-1).endrotation(:,2),pc(n+1).virtual_joint-joint_temp)/norm(joint_temp-pc(n+1).virtual_joint));
%     theta_dis=min(theta_dis,pi/2);
    theta_dis=theta_dis*sign(dot(pc(n+1).virtual_joint-joint_temp,pc(n-1).endrotation(:,1)));
    
   
%     if theta_dis-(-pc(n).theta)<0.0001 && abs(theta_dis)>0.0001
% %         rand('seed',0);
% %         theta_dis=theta_dis+random('Normal',0,0.01,1,1);
% theta_dis=theta_dis+random('Uniform',-0.0001,0.0001,1,1);
%     end

    
    pc(n).theta=theta_dis;
    pc(n).origin=pc(n-1).endtip;
    pc(n).rotation=pc(n-1).endrotation;
	pc(n)=pc(n).continuum_plot_update;
    pause(0.1);
    %     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\学习\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
%     
    set(l1,'LineStyle','none');
    set(l2,'LineStyle','none');
    set(joint,'Marker','none');
    pause(0.1);
%     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\学习\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
    
    n=n+1;
    
%     joint_temp=pc(n-1).endtip+pc(n-1).endrotation(:,2)*pc(n).tangent_length;
%     l=line([pc(n-1).endtip(1), joint_temp(1)],[pc(n-1).endtip(2), joint_temp(2)],'LineStyle','--');
% %     theta_dis=acos(dot(pc(n-1).endrotation(:,2),target-joint_temp)/norm(target-joint_temp))*sign(dot(target-joint_temp,pc(n-1).endrotation(:,1)));
% %     theta_dis=acos(dot(pc(n-1).endrotation(:,2),target-joint_temp)/norm(target-joint_temp));

    theta_dis=acos(dot(pc(n-1).endrotation(:,2),[cos(-target_theta+pi/2);sin(-target_theta+pi/2)]));
%     theta_dis=min(theta_dis,pi/2);
    theta_dis=theta_dis*sign(dot([cos(-target_theta+pi/2);sin(-target_theta+pi/2)],pc(n-1).endrotation(:,1)));
    
%     if theta_dis-(-pc(n).theta)<0.0001 && abs(theta_dis)>0.0001
% %         rand('seed',0);
% %         theta_dis=theta_dis+random('Normal',0,0.01,1,1);
% theta_dis=theta_dis+random('Uniform',-0.0001,0.0001,1,1);
%     end    


    pc(n).theta=theta_dis;
    pc(n).origin=pc(n-1).endtip;
    pc(n).rotation=pc(n-1).endrotation;
    pc(n)=pc(n).continuum_plot_update;
    pause(0.1);
    %     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\学习\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
%     
    set(l1,'LineStyle','none');
    set(l2,'LineStyle','none');
    set(joint,'Marker','none');
    pause(0.1);
%     F=getframe(gcf);
%     tmp=F;
%     filename=['D:\学习\fabrik_for_continuum\frame_',num2str(frame_num),'.png'];
%     saveas(gcf,filename,'png');
%     frame_num=frame_num+1;
    
    theta1(iter) = pc(1).theta;
    theta2(iter) = pc(2).theta;
    theta3(iter) = pc(3).theta;
    error(iter+1)=norm(target-pc(end).endtip);
    
end