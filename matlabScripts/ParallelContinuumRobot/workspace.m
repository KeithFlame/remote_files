% calc and view the workspace.
%
% Author: Keith W.
% Ver. 1.0
% Date 25.03.2022

close,clear all;clc;
%% First Construction
SP = getStructurePara_keith;
init_pose = SP.init_pose;
joint_limit = SP.joint_limit;
T_relative_1to2 = [eul2rotm([0 0 pi/6]), [0 5 0]';[0 0 0 1]];
% T_relative_1to2 = [eul2rotm([0 0 pi]), [0 5 0]';[0 0 0 1]];
threshold = [0.01 0.01 1];
frequency = 5000000;
pool_size= zeros(4,4,frequency);
%% 
psi=getPsi(frequency);
tic;
for i =1 : size(psi,1)
    psi1=psi(i,:);
    [T1, S1] = forwardKinematicsandPlotSnake_keith(psi1,1,0);
    T2=init_pose\T1*T_relative_1to2;
    psi2=invKine_keith(T2);
    [T2i, S2] = forwardKinematicsandPlotSnake_keith(psi2,2,0);
    vT=T2i/T_relative_1to2;
    err_p=norm(T1(1:3,4) - vT(1:3,4));
    axang=rotm2axang(vT(1:3,1:3)'*T1(1:3,1:3));
    err_r=180/pi*axang(4);
    err_c=isColliding(S1,S2);
    err=[err_p,err_r, err_c];
    if(max(err - threshold)<0)
        pool_size(:,:,i)=T1;
    end
end
toc;
%%
% t=load('./data/worakspace_0_0_pi6_0_-5_0.mat');
% pool_size=t.pool_size;
% frequency=size(pool_size,3);
P=zeros(3,frequency);
for i = 1:size(pool_size,3)
    if(pool_size(4,4,i)>0)
        P(:,i)=[pool_size(1,4,i),pool_size(2,4,i),pool_size(3,4,i)]';
    end
end
P(:,all(P == 0, 1)) = [];
P=P';
shp=alphaShape(P);
figure;axis equal;grid on;hold on;
plot(shp);
[k,volume] = boundary(P,0.5);
figure;axis equal;grid on;hold on;
trisurf(k,P(:,1),P(:,2),P(:,3),'Facecolor','red','FaceAlpha',0.1)
shading interp;

function psi=getPsi(frequency)
    SP = getStructurePara_keith;
    LS = SP.size_para;
    joint_limit = SP.joint_limit;
    phi = unifrnd(joint_limit(1),joint_limit(2),[1,frequency]);
    L = unifrnd(joint_limit(3),joint_limit(4),[1,frequency]);
    theta1 = unifrnd(joint_limit(5),joint_limit(6),[1,frequency]);
    delta1 = unifrnd(joint_limit(7),joint_limit(8),[1,frequency]);
    theta2 = unifrnd(joint_limit(9),joint_limit(10),[1,frequency]);
    delta2 = unifrnd(joint_limit(11),joint_limit(12),[1,frequency]);
    
    vu=theta1./L;
    u1_limit=pi/2/LS(1);
    t=find(vu>u1_limit);
    theta1(t) = L(t)*u1_limit;
    psi=[phi' L' theta1' delta1' theta2' delta2'];
end

function c = isColliding(S1,S2)
    colliding_distance = 8;
    D=pdist2(S1(1:3,1:end-1)', S2(1:3,1:end-1)','squaredeuclidean');
    c=colliding_distance^2/min(min(D));
end
