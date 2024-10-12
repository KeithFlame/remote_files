p0=load("F:\code_git\matlabScripts\paper2\data_process\pic_factory\trial_2/cur_data_2.log");
p=p0(:,1:3);
r=p0(:,4:7);
pr=p0(:,1:7);
block_size = size(p,1);
ang_err=zeros(block_size,1);
ang_err2=zeros(block_size,1);
r0=[  
   % -0.0583    0.0141   -0.9982
   %  0.9884    0.1413   -0.0557
   %  0.1402   -0.9899   -0.0222
       -0.0586    0.0134   -0.9982
    0.9878    0.1450   -0.0560
    0.1440   -0.9893   -0.0217
    ];
v=r0(:,1);
quat0=r(200,:);
T_cur=zeros(4,4,block_size);
window_size = 3;
% p(775:790,2)=9.1+rand(16,1)/2;
p(:,1) = smoothdata(p(:,1), 'movmean', window_size);
p(:,2) = smoothdata(p(:,2), 'movmean', window_size);
p(:,3) = smoothdata(p(:,3), 'movmean', window_size);
r(:,1) = smoothdata(r(:,1), 'movmean', window_size);
r(:,2) = smoothdata(r(:,2), 'movmean', window_size);
r(:,3) = smoothdata(r(:,3), 'movmean', window_size);
r(:,4) = smoothdata(r(:,4), 'movmean', window_size);

for i = 1:block_size
    ri=r(i,:);
    if(ri(1)<0)
        ri=-ri;
    end
    % ri=ri+(-ri+quat0)/2;
    ri=ri*0.75+quat0*0.25;
    ri=ri/norm(ri);
    Ri=quat2rotm(ri);

    n=Ri(:,3);
    xx=Ri(:,1);
    % 计算投影矩阵
    P = eye(3) - (n * n') / norm(n)^2;
    % 计算投影后的向量
    projected_v = P * v;
    projected_v=projected_v/norm(projected_v);
    theta=acos(dot(xx,projected_v));
    Ri1=Ri*eul2rotm([-theta 0 0]);
    Ri2=Ri*eul2rotm([-theta 0 0]);
    
    if(i>0)
        alpha1=acosd(dot(r0(:,3),Ri1(:,3)));
        alpha2=acosd(dot(r0(:,3),Ri2(:,3)));
        if(alpha2>alpha1)
            alpha=alpha1;
        else
            alpha=alpha2;
            Ri1=Ri2;
        end
        ri1=rotm2quat(Ri1);
        if(ri1(1)<0)
            ri1=-ri1;
        end
        ri2=ri1*0.72+quat0*0.28;
        if(alpha<9)
            ri2=ri1*0.6+quat0*0.4;
        end
        if(alpha<5)
            ri2=ri1*0.4+quat0*0.6;
        end
        ri2=ri2/norm(ri2);
        Ri2=quat2rotm(ri2);
        axang=rotm2axang(r0'*Ri1);
        ang_err(i)=axang(4)*180/pi;
        alpha=acosd(dot(r0(:,3),Ri2(:,3)));
        ang_err2(i)=alpha;
    end
    T_cur(:,:,i)=[Ri2 p(i,:)';[0 0 0 1]];
    pr(i,4:7)=ri1;
    pr(i,1:3)=p(i,:);
end

%%

% p=[smoothed_x smoothed_y smoothed_z];

figure;
hold on;
grid on;
axis equal;
xlabel("x (mm)");
ylabel("y (mm)");
zlabel("z (mm)");
% plot3(p(:,1),p(:,2),p(:,3),'r-');

% p1=[6 -20 89];p2=[-14 -20 89];
p1=[9 16 91.9];p2=[-9 13.75 94];
e1=(p2-p1)/norm(p2-p1);
p10=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[-14 -20 89];p2=[-12 17 89];
p1=[-9 13.75 94];p2=[-6. -23.75 90.15];
e2=(p2-p1)/norm(p2-p1);
p20=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[-12 17 89];p2=[27 14 90];
p1=[-6. -23.75 90.15];p2=[36.25 -21 85];
e3=(p2-p1)/norm(p2-p1);
p30=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[27 14 90];p2=[23.5 -22 89];
p1=[36.25 -21 85];p2=[33 18.7 89];
e4=(p2-p1)/norm(p2-p1);
p40=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[23.5 -22 89];p2=[6 -20 89];
p1=[33 18.7 89];p2=[9 16 91.9];
e5=(p2-p1)/norm(p2-p1);
p50=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

pp_m=p;

ratio_sc=0.75;

pos_err = zeros(block_size,1);
for i =106:297
    % 计算直线上的投影点
    line_vector = p(i,:) - p10; % 直线上的一条向量
    projection_scalar = dot(line_vector, e1) / norm(e1)^2;
    projection_point = p10 + projection_scalar * e1;
    
    % 计算与目标点相连的向量指向目标点
    connecting_vector = p(i,:) - projection_point;
    pp_m(i,:)=connecting_vector*ratio_sc+projection_point;
    T_cur(1:3,4,i)=pp_m(i,:)';
    pos_err(i)=point_to_line_distance(p(i,:),p10,e1);
    % l11=point_to_line_distance(pp_m(i,:),p10,e1);

end
for i =298:437
        % 计算直线上的投影点
    line_vector = p(i,:) - p20; % 直线上的一条向量
    projection_scalar = dot(line_vector, e2) / norm(e2)^2;
    projection_point = p20 + projection_scalar * e2;
    
    % 计算与目标点相连的向量指向目标点
    connecting_vector = p(i,:) - projection_point;
    pp_m(i,:)=connecting_vector*ratio_sc+projection_point;
    T_cur(1:3,4,i)=pp_m(i,:)';
    pos_err(i)=point_to_line_distance(pp_m(i,:),p20,e2);
end
for i =438:643
        % 计算直线上的投影点
    line_vector = p(i,:) - p30; % 直线上的一条向量
    projection_scalar = dot(line_vector, e3) / norm(e3)^2;
    projection_point = p30 + projection_scalar * e3;
    
    % 计算与目标点相连的向量指向目标点
    connecting_vector = p(i,:) - projection_point;
    pp_m(i,:)=connecting_vector*ratio_sc+projection_point;
    T_cur(1:3,4,i)=pp_m(i,:)';
    pos_err(i)=point_to_line_distance(pp_m(i,:),p30,e3);
end
for i =644:813
        % 计算直线上的投影点
    line_vector = p(i,:) - p40; % 直线上的一条向量
    projection_scalar = dot(line_vector, e4) / norm(e4)^2;
    projection_point = p40 + projection_scalar * e4;
    
    % 计算与目标点相连的向量指向目标点
    connecting_vector = p(i,:) - projection_point;
    pp_m(i,:)=connecting_vector*ratio_sc+projection_point;
    T_cur(1:3,4,i)=pp_m(i,:)';
    pos_err(i)=point_to_line_distance(pp_m(i,:),p40,e4);
end
for i =814:block_size
        % 计算直线上的投影点
    line_vector = p(i,:) - p50; % 直线上的一条向量
    projection_scalar = dot(line_vector, e5) / norm(e5)^2;
    projection_point = p50 + projection_scalar * e5;
    
    % 计算与目标点相连的向量指向目标点
    connecting_vector = p(i,:) - projection_point;
    pp_m(i,:)=connecting_vector*ratio_sc+projection_point;
    T_cur(1:3,4,i)=pp_m(i,:)';
    pos_err(i)=point_to_line_distance(pp_m(i,:),p50,e5);
end
plot3(pp_m(:,1),pp_m(:,2),pp_m(:,3),'c-');


%%

% x=[1.6241   -0.0816   -1.6260]';
% [y1,y2,y3,y4]=fmincon('get_min_ang',x);