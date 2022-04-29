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
    
    % end-effector
    [stator,rotor] = getEffector;
    effector.stator = stator;
    effector.rotor = rotor;

    sp0.size_para = size_para;
    sp0.stiffness_para = stiffness_para;
    sp0.assembly_para = assembly_para;
    sp0.init_pose = init_pose;
    sp0.joint_limit = joint_limit;
    sp0.effector = effector;
end

SP=sp0;
end

function [stator,rotor] = getEffector
    [x1,y1,z1]=stlread_keith('./end_effector/nh_1.STL');
    [x2,y2,z2]=stlread_keith('./end_effector/nh_0.STL');
    block_size1=max(size(x1));
    block_size2=max(size(x2));
    R1=eul2rotm([ 0, 0 ,-pi/2]);
    R2=eul2rotm([ 0, 0 ,pi]);
    for j = 1:3
        for i = 1:block_size1
            p1=[x1(j,i); y1(j,i);z1(j,i)];
            p1=R1*p1;
            x1(j,i)=p1(1);y1(j,i)=p1(2);z1(j,i)=p1(3);
        end
        for i = 1:block_size2
            p2=[x2(j,i); y2(j,i);z2(j,i)];
            p2=R2*p2;
            x2(j,i)=p2(1);y2(j,i)=p2(2);z2(j,i)=p2(3);
        end
    end
    x1=x1-1.8;
    % % % % z1=z1+17.98;y1=y1-2.4;
    z1=z1+8.98;y1=y1-1.23;
    z2=z2+17.98;y2=y2+3;x2=x2-2.63685;
    p = [0, -1.17, 9,0,0,0,0,0,0]';
    stator = [x2;y2;z2];
    rotor = [[x1;y1;z1] p];
end