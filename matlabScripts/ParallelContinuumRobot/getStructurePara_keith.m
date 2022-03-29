function SP=getStructurePara_keith(is_refreshed)
% This is a function to get the structure parameters of a 2-segment continuum robot. 
% And if isfreshed (input parameter) is null, mean read only.
% Else need to read/refresh the structure parameter in the file.
% 
% Author Keith W.
% Ver.1.0
% Date 03.24.2022

if(nargin==0)
    is_refreshed=0;
end

persistent sp0;
if(is_refreshed == 1|| isempty(sp0))
    % L1 Lr L2 Lg Lstem gamma1 gamma3
    size_para=load('./structurePara/size_para.raw');
    % K1 K2 zeta
    stiffness_para=load('./structurePara/stiffness_para.raw');
    
    % diamter_L1 diameter_Lr diameter_L2 number_L1 number_L2 first_L1
    % first_L2 spacer_interval
    assembly_para=load('./structurePara/assembly_para.raw');
    
    % roll pitch yaw x y z
    init_pose0=load('./structurePara/init_pose.raw');
    init_r=eul2rotm(init_pose0(1:3));
    init_p=reshape(init_pose0(4:6),[3 1]);
    init_pose=[init_r init_p;[0 0 0 1]];

    % joint limit
    % phi_min phi_max L_min L_max theta1_min theta1_max delta1_min
    % delta1_max theta2_min theta2_max delta2_min delta2_max
    joint_limit = load('./structurePara/joint_limit.raw');

    sp0.size_para = size_para;
    sp0.stiffness_para = stiffness_para;
    sp0.assembly_para = assembly_para;
    sp0.init_pose = init_pose;
    sp0.joint_limit = joint_limit;
end

SP=sp0;
end


