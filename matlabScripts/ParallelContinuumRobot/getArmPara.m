function AP = getArmPara(arm_serial, port)
%   this is a function to get a arm para
%   input1 is number serials
%   
%   Author Keith W.
%   Ver. 1.0
%   Date 05.02.2022
%
% added vision_field
%
%   Author Keith W.
%   Ver. 1.1
%   Date 05.05.2022
%
% added new needle holder, add wusundazhuaqian
%
%   Author Keith W.
%   Ver. 1.2
%   Date 05.06.2022
%
% added statics parameter
%   Author Keith W.
%   Ver. 2.0 
%   Date 07.29.2022

if(nargin==0)
    arm_serial=111;
    port = 3;
end
if(nargin == 1)
    port = 1;
end
%% 判断臂的序列号是否重复

persistent arm_repo;
if(isempty(arm_repo))
    arm_repo0 = [];
else
    arm_repo0 = arm_repo;
end
for ir = 1:size(arm_repo,1)
    if(arm_serial == arm_repo(ir))
        error("repeat ARM !!!");
    end
end
arm_repo0 = [arm_repo0;arm_serial];
arm_repo = arm_repo0;

armtype_flag = mod(arm_serial,1000);
endoscope_flag = 111;
needle_holder = 112;
wusun_dazhuaqian = 113;
if((armtype_flag == endoscope_flag && port ~= 3)||(armtype_flag ~= endoscope_flag && port == 3))
    error("endoscope NOT others need to arrange on port 3!!!");
end
%% 判断该序列号的臂是否存在
name=['./structurePara/arm_',num2str(arm_serial)];
if  ~exist(name,'dir')%~exist(name,'file')
   error('Error! \nno  file folder named %s !!!',name);
end
name0=[name,'/size_para.raw'];
if  ~exist(name0,'file')
   error('Error! \nno  file named %s !!!',name0);
end
name0=[name,'/stiffness_para.raw'];
if ~exist(name0,'file')
   error('Error! \nno  file named %s !!!',name0);
end

%% 载入臂的信息
% 判断是否为视觉臂
if (armtype_flag == endoscope_flag)
    is_endoscope = true;
else
    is_endoscope = false;
end

% L1 Lr L2 Lg Lstem gamma1 gamma3
name0=[name,'/size_para.raw'];
size_para=load(name0);
% K1 K2 zeta
name0=[name,'/stiffness_para.raw'];
stiffness_para=load(name0);

% material parameter
% E poissiom_rate
material_para=load('./structurePara/material_para.raw');

if(~is_endoscope)
    % diamter_L1 diameter_Lr diameter_L2 number_L1 number_L2 first_L1
    % first_L2 spacer_interval d1 d2
    assembly_para=load('./structurePara/tool_assembly_para.raw');

    % joint limit
    % phi_min phi_max L_min L_max theta1_min theta1_max delta1_min
    % delta1_max theta2_min theta2_max delta2_min delta2_max
    joint_limit = load('./structurePara/tool_joint_limit.raw');
else
    assembly_para=load('./structurePara/endoscope_assembly_para.raw');
    joint_limit = load('./structurePara/endoscope_joint_limit.raw');
end
% end-effector
if(armtype_flag == needle_holder)
    [stator,rotor] = getNeedleHolder;
    effector.stator = stator;
    effector.rotor = rotor;
end
if(armtype_flag == wusun_dazhuaqian)
    [stator,rotor] = getWusunDazhuaqian;
    effector.stator = stator;
    effector.rotor = rotor;
end
if(armtype_flag == endoscope_flag)
    % camera
    [camera,vision_field] = getCamera;
    effector.rotor =vision_field;
    effector.stator = camera;
    % endoscope is set to port3, only.
    port = 3;
end


distribution_para = getDistribution(assembly_para,is_endoscope);

ap0.size_para = size_para;
ap0.stiffness_para = stiffness_para;
ap0.material_para = material_para;
ap0.assembly_para = assembly_para;
ap0.joint_limit = joint_limit;
ap0.effector = effector;
ap0.port = port;
ap0.distribution_para = distribution_para;
AP=ap0;

end


function [stator,rotor] = getNeedleHolder
    [x1,y1,z1]=stlread_keith('./end_effector/needle_holder_rotor_v2.STL');
    [x2,y2,z2]=stlread_keith('./end_effector/needle_holder_stator_v2.STL');
    block_size1=max(size(x1));
    block_size2=max(size(x2));
    R1=eul2rotm([ 0, 0 ,-pi]);
    R2=eul2rotm([ 0, 0 ,pi]);
    for j = 1:3
        for ir = 1:block_size1
            p1=[x1(j,ir); y1(j,ir);z1(j,ir)];
            p1=R1*p1;
            x1(j,ir)=p1(1);y1(j,ir)=p1(2);z1(j,ir)=p1(3);
        end
        for ir = 1:block_size2
            p2=[x2(j,ir); y2(j,ir);z2(j,ir)];
            p2=R2*p2;
            x2(j,ir)=p2(1);y2(j,ir)=p2(2);z2(j,ir)=p2(3);
        end
    end
    x1=x1-1.8;
    % % % % z1=z1+17.98;y1=y1-2.4;
    z1=z1+15.25;y1=y1+2.8;
    z2=z2+24.25;y2=y2+3;x2=x2-2.63685;
    p = [0, -1.17, 9,0,0,0,0,0,1]';
    stator = [x2;y2;z2];
    rotor = [[x1;y1;z1] p];
end

function [camera,vision_field] = getCamera
    [x2,y2,z2]=stlread_keith('./end_effector/camera.STL');
    y2=y2-5;x2=x2-5;z2=z2+0.001;
    camera = [x2;y2;z2];
    [x2,y2,z2] = stlread_keith('./end_effector/vision_field_v2.STL');
    y2 = y2 - 4.9051;
    x2 = x2 - 11.1555;
    z2 = z2 +15.1;
    vision_field = [x2;y2;z2 ];
    vision_field = [vision_field zeros(9,1)];
end

function [stator,rotor] = getWusunDazhuaqian
    [x1,y1,z1]=stlread_keith('./end_effector/wusundazhuaqian_stator.STL');
    [x2,y2,z2]=stlread_keith('./end_effector/wusundazhuaqian_rotor.STL');
    block_size1=max(size(x1));
    block_size2=max(size(x2));
    R1=eul2rotm([ pi/2, -pi/2 ,0]);
    R2=eul2rotm([ pi/2, -pi/2 ,0]);
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
    y1=y1+12.8939;x1=x1+3.25;
    y2=y2+12.8939;x2=x2+3.2;z2=z2-7.405;
    p = [0, 0, 11.7041,0,0,0,0,0,2]';
    stator = [x1;y1;z1];
    rotor = [[x2;y2;z2] p];
end
function distribution_para = getDistribution(assembly_para,is_endoscope)
    rho1 = assembly_para(1)/2;
    rho2 = assembly_para(3)/2;
    ai1 = assembly_para(6);
    ai2 = assembly_para(7);
    e3 = [0 0 1]';
    if(is_endoscope)
        r11=[cos(ai1) sin(ai1) 0]'*rho1;
        r12=[cos(ai1 + pi/2) sin(ai1 + pi/2) 0]'*rho1;
        r13=[cos(ai1 + pi) sin(ai1 + pi) 0]'*rho1;
        r14=[cos(ai1 + pi/0.75) sin(ai1 + pi/0.75) 0]'*rho1;
        r21=[cos(ai2) sin(ai2) 0]'*rho2;
        r22=[cos(ai2 + pi/2) sin(ai2 + pi/2) 0]'*rho2;
        r23=[cos(ai2 + pi) sin(ai2 + pi) 0]'*rho2;
        r24=[cos(ai2 + pi/0.75) sin(ai2 + pi/0.75) 0]'*rho2;
%         Q1 = [S(r11)*e3 S(r12)*e3 S(r13)*e3 S(r14)*e3];
%         Q2 = [S(r21)*e3 S(r22)*e3 S(r23)*e3 S(r24)*e3];
        Q1 = [r11 r12 r13 r14];
        Q2 = [r21 r22 r23 r24];
    else
        r11=[cos(ai1) sin(ai1) 0]'*rho1;
        r12=[cos(ai1 + pi/2) sin(ai1 + pi/2) 0]'*rho1;
        r13=[cos(ai1 + pi) sin(ai1 + pi) 0]'*rho1;
        r14=[cos(ai1 + pi/2*3) sin(ai1 + pi/2*3) 0]'*rho1;
        gap_r2 = (pi/2 - ai2 * 2) / 3;
        r21=[cos(1 * ai2 + gap_r2 * 0) sin(1 * ai2 + gap_r2 * 0) 0]'*rho2;
        r22=[cos(1 * ai2 + gap_r2 * 1) sin(1 * ai2 + gap_r2 * 1) 0]'*rho2;
        r23=[cos(1 * ai2 + gap_r2 * 2) sin(1 * ai2 + gap_r2 * 2) 0]'*rho2;
        r24=[cos(1 * ai2 + gap_r2 * 3) sin(1 * ai2 + gap_r2 * 3) 0]'*rho2;
        r25=[cos(3 * ai2 + gap_r2 * 3) sin(3 * ai2 + gap_r2 * 3) 0]'*rho2;
        r26=[cos(3 * ai2 + gap_r2 * 4) sin(3 * ai2 + gap_r2 * 4) 0]'*rho2;
        r27=[cos(3 * ai2 + gap_r2 * 5) sin(3 * ai2 + gap_r2 * 5) 0]'*rho2;
        r28=[cos(3 * ai2 + gap_r2 * 6) sin(3 * ai2 + gap_r2 * 6) 0]'*rho2;
        r29=[cos(5 * ai2 + gap_r2 * 6) sin(5 * ai2 + gap_r2 * 6) 0]'*rho2;
        r2a=[cos(5 * ai2 + gap_r2 * 7) sin(5 * ai2 + gap_r2 * 7) 0]'*rho2;
        r2b=[cos(5 * ai2 + gap_r2 * 8) sin(5 * ai2 + gap_r2 * 8) 0]'*rho2;
        r2c=[cos(5 * ai2 + gap_r2 * 9) sin(5 * ai2 + gap_r2 * 9) 0]'*rho2;
        r2d=[cos(7 * ai2 + gap_r2 * 9) sin(7 * ai2 + gap_r2 * 9) 0]'*rho2;
        r2e=[cos(7 * ai2 + gap_r2 * 10) sin(7 * ai2 + gap_r2 * 10) 0]'*rho2;
        r2f=[cos(7 * ai2 + gap_r2 * 11) sin(7 * ai2 + gap_r2 * 11) 0]'*rho2;
        r2g=[cos(7 * ai2 + gap_r2 * 12) sin(7 * ai2 + gap_r2 * 12) 0]'*rho2;
%         Q1 = [S(r11)*e3 S(r12)*e3 S(r13)*e3 S(r14)*e3];
%         Q2=[S(r21)*e3 S(r22)*e3 S(r23)*e3 S(r24)*e3 ...
%             S(r25)*e3 S(r26)*e3 S(r27)*e3 S(r28)*e3 ...
%             S(r29)*e3 S(r2a)*e3 S(r2b)*e3 S(r2c)*e3 ...
%             S(r2d)*e3 S(r2e)*e3 S(r2f)*e3 S(r2g)*e3];
        Q1 = [r11 r12 r13 r14];
        Q2 = [r21 r22 r23 r24 ...
              r25 r26 r27 r28 ...
              r29 r2a r2b r2c ...
              r2d r2e r2f r2g];
    end
    distribution_para.Q1 = Q1;
    distribution_para.Q2 = Q2;

end

