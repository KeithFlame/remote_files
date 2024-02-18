function pose = getWorkspaceTargets(d8,N)
% input: N split each angle dimension to N pieces. 
% output: target bunch

if( nargin == 0)
    d8 = -40;
    N = 26;
end
if(nargin == 1)
    N = 26;
end

O_RCM=[0 -232.5+100 100]';
max_angle = 75 * pi / 180;
piece=max_angle/(N-1);
pose = zeros(6,2*N*N);
iter = 1;
for i = 1:N
    for j = 1:N
        for m = [-0.001 -200]
            Fy=piece*(i-1);Pz=piece*(j-1);Jj=m;
            Pj_XY1=Jj*cos(Fy);
            RCM_TT=[-Pj_XY1*cos(Pz) -Pj_XY1*sin(Pz) Jj*sin(Fy)]';  
%             PO1=[58 -572.5+50 -85]';%方案1
%             PO2=PO1+d1*[0 0 1]';
            
            %求机械臂末端姿态
            Dirction_RT=RCM_TT/norm(RCM_TT);%工具的轴线方向
            L=d8+650.1800;
            RCM_w=RCM_TT-L*Dirction_RT;%w点(腕部)在RCM坐标系下的坐标(坐标系与世界坐标系对齐)
            PO7=RCM_w+O_RCM;
            Z_W=[0 0 1]';
%             O7R=O_RCM-PO7;
            Z_T1=Dirction_RT;%工具Z轴方向(忽略工具Z方向转动)
            n_TRZ=cross(Z_W,Z_T1);
            Y_T1=n_TRZ/norm(n_TRZ);%工具X轴方向(忽略工具Z方向转动)
            X_T1=cross(Y_T1,Z_T1);%工具Y轴方向(忽略工具Z方向转动)
            ROT=[X_T1 Y_T1 Z_T1];%旋转矩阵
            pose(1:3,iter) = PO7;
            axang = rotm2axang(ROT);
            pose(4:6,iter) = axang(4)*axang(1:3)'*180/pi;
            iter = iter + 1;

        end
    end
end

end

