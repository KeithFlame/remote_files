function SP=getStructurePara_keith(is_refreshed)
% This is a function to get the structure parameters of a 2-segment continuum robot. 
% And if isfreshed (input parameter) is null, mean read only.
% Else need to read/refresh the structure parameter in the file.
% 
% Author Keith W.
% Ver.1.0
% Date 03.24.2022
%
% make a better effector model to place the old one
%
% Author Keith W.
% Ver. 1.1
% Date 04.29.2022
%
% delete and add armpara function

if(nargin==0)
    is_refreshed=0;
end

persistent sp0;
if(is_refreshed == 1|| isempty(sp0))
% % % %     % L1 Lr L2 Lg Lstem gamma1 gamma3
% % % %     tool_size_para=load('./structurePara/tool_size_para.raw');
% % % %     % K1 K2 zeta
% % % %     tool_stiffness_para=load('./structurePara/tool_stiffness_para.raw');
% % % %     
% % % %     % diamter_L1 diameter_Lr diameter_L2 number_L1 number_L2 first_L1
% % % %     % first_L2 spacer_interval
% % % %     tool_assembly_para=load('./structurePara/tool_assembly_para.raw');
% % % %     
% % % %     
% % % %     % joint limit
% % % %     % phi_min phi_max L_min L_max theta1_min theta1_max delta1_min
% % % %     % delta1_max theta2_min theta2_max delta2_min delta2_max
% % % %     tool_joint_limit = load('./structurePara/tool_joint_limit.raw');

    % roll pitch yaw x y z
    init_pose0=load('./structurePara/init_pose.raw');
    init_pose=zeros(4,4,4);
    for i = 1 :size(init_pose0,1)
        init_r=eul2rotm(init_pose0(i,1:3));
        init_p=reshape(init_pose0(i,4:6),[3 1]);
        init_pose(:,:,i)=[init_r init_p;[0 0 0 1]];
    end

    
    % needles
    needle.l=0.5;
    needle.r=15;
    needle.d=3;
    
    % trocar 
    trocar = getTrocar;

    sp0.init_pose = init_pose;
    sp0.needle = needle;
    sp0.trocar = trocar;

end

SP=sp0;
end

function trocar = getTrocar
    [x2,y2,z2]=stlread_keith('./trocar_end_face/trocar_end_face.STL');
    y2=y2-13;x2=x2-13;z2=z2-1;
    trocar = [x2;y2;z2];
end

