pc1=planar_continuum(0,100,[0;0],eye(2),[0.8,0,0]);

pc2=planar_continuum(0,100,pc1.endtip,eye(2),[0,0.8,0]);

pc3=planar_continuum(0,100,pc2.endtip,eye(2),[0,0,0.8]);

target=[100;250];
scatter(target(1),target(2),'filled');

% figure(1)
axis equal 
axis([-150 150 -10 350])
hold on 
pc1=pc1.continuum_plot;
pc2=pc2.continuum_plot;
pc3=pc3.continuum_plot;



%%%% backward %%%%%%%%
l1=line([target(1), pc3.virtual_joint(1)],[target(2), pc3.virtual_joint(2)],'LineStyle','--');

% theta_dis=acos(dot(pc3.virtual_joint-target,-[pc3.rotation(1,2);pc3.rotation(2,2)])/norm(pc3.virtual_joint-target))*sign(dot([-pc3.rotation(1,2);-pc3.rotation(2,2)],[cos(-pi/2),-sin(-pi/2);sin(-pi/2),cos(-pi/2)]*(pc3.virtual_joint-target)));
theta_dis=acos(dot(pc3.virtual_joint-target,-[pc3.rotation(1,2);pc3.rotation(2,2)])/norm(pc3.virtual_joint-target))*sign(dot([pc3.rotation(1,1);pc3.rotation(2,1)],(pc3.virtual_joint-target)));
rotation_temp=[[cos(-pi/2),-sin(-pi/2);sin(-pi/2),cos(-pi/2)]*(pc3.virtual_joint-target)/norm(pc3.virtual_joint-target),(pc3.virtual_joint-target)/norm(pc3.virtual_joint-target)];

pc3.theta=theta_dis;
pc3.origin=target;
pc3.rotation=rotation_temp;

pc3=pc3.continuum_plot_update;
set(l1,'LineStyle','none');

joint_temp=pc3.endtip+pc3.endrotation(:,2)*pc2.tangent_length;
l2=line([pc3.endtip(1), joint_temp(1)],[pc3.endtip(2), joint_temp(2)],'LineStyle','--');
theta_dis=acos(dot(pc3.endrotation(:,2),pc1.virtual_joint-joint_temp)/norm(pc1.virtual_joint-joint_temp))*sign(dot(pc1.virtual_joint-joint_temp,pc3.endrotation(:,1)));
pc2.theta=theta_dis;
pc2.origin=pc3.endtip;
pc2.rotation=pc3.endrotation;
pc2=pc2.continuum_plot_update;
set(l2,'LineStyle','none');


theta_dis=acos(dot(-pc2.endrotation(:,2),pc1.rotation(:,2)))*sign(dot(pc1.rotation(:,1),-pc2.endrotation(:,2)));
pc1.theta=theta_dis;
pc1=pc1.continuum_plot_update;

%%%% forward %%%%%%%%
joint_temp=pc1.endtip+pc1.endrotation(:,2)*pc2.tangent_length;
l3=line([pc1.endtip(1), joint_temp(1)],[pc1.endtip(2), joint_temp(2)],'LineStyle','--');

theta_dis=acos(dot(pc1.endrotation(:,2),pc3.virtual_joint-joint_temp)/norm(joint_temp-pc3.virtual_joint))*sign(dot(pc3.virtual_joint-joint_temp,pc1.endrotation(:,1)));
pc2.theta=theta_dis;
pc2.origin=pc1.endtip;
pc2.rotation=pc1.endrotation;
pc2=pc2.continuum_plot_update;
set(l2,'LineStyle','none');

joint_temp=pc2.endtip+pc2.endrotation(:,2)*pc3.tangent_length;
l4=line([pc2.endtip(1), joint_temp(1)],[pc2.endtip(2), joint_temp(2)],'LineStyle','--');
theta_dis=acos(dot(pc2.endrotation(:,2),target-joint_temp)/norm(target-joint_temp))*sign(dot(target-joint_temp,pc2.endrotation(:,1)));
pc3.theta=theta_dis;
pc3.origin=pc2.endtip;
pc3.rotation=pc2.endrotation;
pc3=pc3.continuum_plot_update;
set(l2,'LineStyle','none');