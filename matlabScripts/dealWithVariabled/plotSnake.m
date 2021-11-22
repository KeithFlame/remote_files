function plotSnake(S0,S1,S2,S3,S4,S5,S6,Config_plot,T_config)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.11.2021
% Ver. 1.0
% as name said,drawing the arm.
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%%
hold on;
plot3(S0(1,:), S0(2,:),S0(3,:),'b',LineWidth=2);
plot3(S1(1,:), S1(2,:),S1(3,:),'c',LineWidth=2);
plot3(S2(1,:), S2(2,:),S2(3,:),'r',LineWidth=2);
plot3(S3(1,:), S3(2,:),S3(3,:),'g',LineWidth=2);
plot3(S4(1,:), S4(2,:),S4(3,:),'b',LineWidth=2);
plot3(S5(1,:), S5(2,:),S5(3,:),'c',LineWidth=2);
plot3(S6(1,:), S6(2,:),S6(3,:),'r',LineWidth=2);
alpha(1.0);
[r_L1, ~, r_Lr, ~, r_L2, ~, r_Lg, ~, ~,r_Lstem]=getToolArmStructureParameter;
%% seg1
Lstem_l=Config_plot(1,1);
theta_Lstem=Config_plot(2,1);
if(Lstem_l~=0)
    seg_color=	[1 0 1];
    plotTorus(Lstem_l,r_Lstem,theta_Lstem,seg_color, T_config(:,:,1));
end

%% seg2
L1t_l=Config_plot(1,2);
theta_L1t=Config_plot(2,2);
if(L1t_l~=0)
    seg_color=[0 0 1];
    plotTorus(L1t_l,r_L1,theta_L1t,seg_color, T_config(:,:,2));
end

%% seg2
L1_l=Config_plot(1,3);
theta_L1=Config_plot(2,3);
if(L1_l~=0)
    seg_color=[1 0 0];
    plotTorus(L1_l,r_L1,theta_L1,seg_color, T_config(:,:,3));
end

%% seg3
Lr_l=Config_plot(1,4);
theta_Lr=Config_plot(2,4);
if(Lr_l~=0)
    seg_color=[0 1 0];
    plotTorus(Lr_l,r_Lr,theta_Lr,seg_color, T_config(:,:,4));
end

%% seg4
L2t_l=Config_plot(1,5);
theta_L2t=Config_plot(2,5);
if(L2t_l~=0)
    seg_color=[0 0 1];
    plotTorus(L2t_l,r_L2,theta_L2t,seg_color, T_config(:,:,5));
end

%% seg4
L2_l=Config_plot(1,6);
theta_L2=Config_plot(2,6);
if(L2_l~=0)
    seg_color=[0 1 1];
    plotTorus(L2_l,r_L2,theta_L2,seg_color, T_config(:,:,6));
end

%% seg5
Lg_l=Config_plot(1,7);
theta_Lg=Config_plot(2,7);
if(Lg_l~=0)
    seg_color=[0 0 0];
    plotTorus(Lg_l,r_Lg,theta_Lg,seg_color, T_config(:,:,7));
end
end
