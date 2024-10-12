broy_cur_data = load('C:\Users\keith\Desktop\123\image\pic\date0321\data\data_cur.log');
broy_tar_data = load('C:\Users\keith\Desktop\123\image\pic\date0321\data\data_tar.log');

static_cur_data = load('C:\Users\keith\Desktop\123\image\pic\date0419\data\data_cur.log');
static_tar_data = load('C:\Users\keith\Desktop\123\image\pic\date0419\data\data_tar.log');

%% data pre-process
p_broy_cur = broy_cur_data(:,1:3);
r_broy_cur = broy_cur_data(:,4:7);
p_broy_tar = broy_tar_data(1,1:3);
r_broy_tar = broy_tar_data(1,4:7);
dPt_broy = getPositionError(p_broy_cur,p_broy_tar);
dPb_broy = getPositionError(p_broy_cur,p_broy_cur(1,1:3));
dp_broy = dPt_broy(:,4);
dAAt_broy = getAngleError(r_broy_cur,r_broy_tar);
dAAb_broy = getAngleError(r_broy_cur,r_broy_cur(1,:));
dA_broy = dAAt_broy(:,4);
p_broy_beg = broy_cur_data(1,1:3);

p_static_cur = static_cur_data(:,1:3);
r_static_cur = static_cur_data(:,4:7);
p_static_tar = static_tar_data(1,1:3);
r_static_tar = static_tar_data(1,4:7);
dPt_static = getPositionError(p_static_cur,p_static_tar);
dPb_static = getPositionError(p_static_cur,p_static_cur(1,1:3));
dp_static = dPt_static(:,4);
dAAt_static = getAngleError(r_static_cur,r_static_tar);
dAAb_static = getAngleError(r_static_cur,r_static_cur(1,:));
dA_static = dAAt_static(:,4);
p_static_beg = static_cur_data(1,1:3);
% T_broy_tar = eye(4);
% T_broy_tar(1:3,1:3)=quat2rotm(r_broy_cur(1,:)/norm(r_broy_tar(1,:)));
% T_broy_tar(1:3,4)=p_broy_tar';
% T_broy_beg = eye(4);
% T_broy_beg(1:3,1:3)=quat2rotm(r_broy_cur(1,:)/norm(r_broy_cur(1,:)));
% T_broy_beg(1:3,4)=p_broy_cur(1,:)';
block_size = size(p_broy_cur,1);
% p_broy_dis = (T_broy_tar(1:3,4)-T_broy_cur(1:3,4))';
% 
% 
T_static_tar = eye(4);
T_static_tar(1:3,1:3)=quat2rotm(r_static_tar/norm(r_static_tar));
T_static_tar(1:3,4)=p_static_tar';
T_static_beg = eye(4);
T_static_beg(1:3,1:3)=quat2rotm(r_static_cur(1,:)/norm(r_static_cur(1,:)));
T_static_beg(1:3,4)=p_static_cur(1,:)';

% ratio = p_static_dis./p_broy_dis;
p_broy_st = zeros(block_size,3);
dis_broy = norm(p_broy_beg-p_broy_tar);
dis_static = norm(p_static_beg - p_static_tar);
ratio = dis_static/dis_broy;
e_broy = (p_broy_beg - p_broy_tar)/dis_broy;
e_static = (p_static_beg - p_static_tar)/dis_static;
flag = 1;
for i = 1:block_size
    pi = p_broy_cur(i,:);
    p0 = pointToLineProjection(pi,p_broy_beg,p_broy_tar);
    e=pi-p0';

    d = norm(p_broy_beg-p0');
    ed = (p_broy_beg-p0)/d;
    if(norm(ed-e_broy)>0.1)
        flag = -1;
    end
    % d = e*ratio;
    p0s = p_static_beg+d*ratio*e_static*flag;
    p_broy_st(i,:) = p0s+e;
end


%% plot
figure(1);
% dp_static(end-2:end)=dp_static(end-2:end)/2;
block_size2=size(dp_static,1);
xx=(1:0.2:block_size2)';
dp_statics=interp1(1:block_size2,dp_static,xx,'linear');
hold on; grid on;
xlabel("iteration");ylabel("error (mm)");
title("method in [177] v.s. proposed method");
set(gca, 'FontSize', 30);
set(gca,'FontName','Times New Roman');
plot(dp_broy,'r')
plot(dp_statics,'go-')
legend("method in [177]","proposed method");

figure(2);
% dA_static(end-2:end)=dA_static(end-2:end)/2;
dA_statics=interp1(1:block_size2,dA_static,xx,'linear');
hold on; grid on;
xlabel("iteration");ylabel("error (°)");
title("method in [177] v.s. proposed method");
set(gca, 'FontSize', 30);
set(gca,'FontName','Times New Roman');
plot(dA_broy/2,'r')
plot(dA_statics,'go-')

figure(3);
hold on; grid on;axis equal;
xlabel("x (mm)");ylabel("y (mm)");zlabel("z (mm)");
title("method in [177] v.s. proposed method");
set(gca, 'FontSize', 40);
set(gca,'FontName','Times New Roman');
p_static_cur1=interp1(1:block_size2,p_static_cur(:,1),xx,'linear');
p_static_cur2=interp1(1:block_size2,p_static_cur(:,2),xx,'linear');
p_static_cur3=interp1(1:block_size2,p_static_cur(:,3),xx,'linear');


plot3(p_broy_st(:,1),p_broy_st(:,2),p_broy_st(:,3),'r.-')
plot3(p_static_cur1,p_static_cur2,p_static_cur3,'go-',LineWidth=3)
plotCoord_keith(T_static_beg,1,2);
plotCoord_keith(T_static_tar,1,1);

%% function 
function dA =getAngleError(quat1,quat2)

block_size = size(quat1,1);
dA = zeros(block_size,4);
quat2=quat2/norm(quat2);
for i = 1: block_size
    r_t = quat2rotm(quat2);
    q_c = quat1(i,:);
    q_c = q_c/norm(q_c);
    r_c = quat2rotm(q_c);
    axang = rotm2axang(r_t'*r_c);
    dA(i,1:3)=axang(1:3);
    dA(i,4)=axang(4)*180/pi;
end
end

function dP =getPositionError(p1,p2)

block_size = size(p1,1);
dP = zeros(block_size,4);
% quat2=quat2/norm(quat2);
    for i = 1: block_size
        % r_t = quat2rotm(quat2);
        % q_c = quat1(i,:);
        % q_c = q_c/norm(q_c);
        % r_c = quat2rotm(q_c);
        % axang = rotm2axang(r_t'*r_c);
        pi=p1(i,:);
        dP(i,4)=norm(pi-p2);
        dP(i,1:3)=(pi-p2);
        
    end
end

function P_proj = pointToLineProjection(P0, P1, P2)
    % P0: 投影点 (x0, y0, z0)
    % P1: 直线上的第一个点 (x1, y1, z1)
    % P2: 直线上的第二个点 (x2, y2, z2)

    % 将点转换为列向量
    P0 = P0(:);
    P1 = P1(:);
    P2 = P2(:);
    
    % 计算直线方向向量
    P21 = P2 - P1;
    
    % 计算向量 P1P0
    P1P0 = P0 - P1;
    
    % 计算投影系数
    t = dot(P1P0, P21) / dot(P21, P21);
    
    % 计算投影点
    P_proj = P1 + t * P21;
end