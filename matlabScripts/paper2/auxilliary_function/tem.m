p0=load("temp/data_40_40_2.log");
p=p0(:,1:3);
r=p0(:,4:7);
block_size = size(p,1);
ang_err=zeros(block_size,1);
r0=[  
    0.0505    0.0094   -0.9987
    0.9972    0.0540    0.0510
    0.0544   -0.9985   -0.0067];
T_cur=zeros(4,4,block_size);
for i = 1:block_size
    ri=r(i,:);
    Ri=quat2rotm(ri);
    axang=rotm2axang(r0'*Ri);
    if(i>157)
        ang_err(i)=axang(4)*180/pi;
    end
    T_cur(:,:,i)=[Ri p(i,:)';[0 0 0 1]];
end

%%
window_size = 7;
smoothed_x = smoothdata(p(:,1), 'movmean', window_size);
smoothed_y = smoothdata(p(:,2), 'movmean', window_size);
smoothed_z = smoothdata(p(:,3), 'movmean', window_size);
p=[smoothed_x smoothed_y smoothed_z];

figure;
hold on;
grid on;
axis equal;
xlabel("x (mm)");
ylabel("y (mm)");
zlabel("z (mm)");
plot3(p(:,1),p(:,2),p(:,3),'r-');

% p1=[6 -20 89];p2=[-14 -20 89];
p1=[7.75 15.5 90];p2=[-11 17 90];
e1=(p2-p1)/norm(p2-p1);
p10=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[-14 -20 89];p2=[-12 17 89];
p1=[-11 17 90];p2=[-14.5 -19.5 87];
e2=(p2-p1)/norm(p2-p1);
p20=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[-12 17 89];p2=[27 14 90];
p1=[-14.5 -19.5 87];p2=[24 -23 87];
e3=(p2-p1)/norm(p2-p1);
p30=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[27 14 90];p2=[23.5 -22 89];
p1=[24 -23 87];p2=[27.5 14 90];
e4=(p2-p1)/norm(p2-p1);
p40=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[23.5 -22 89];p2=[6 -20 89];
p1=[27.5 14 90];p2=[7.75 15.5 90];
e5=(p2-p1)/norm(p2-p1);
p50=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

pos_err = zeros(block_size,1);
for i =157:230
    pos_err(i)=point_to_line_distance(p(i,:),p10,e1);
end
for i =231:415
    pos_err(i)=point_to_line_distance(p(i,:),p20,e2);
end
for i =416:572
    pos_err(i)=point_to_line_distance(p(i,:),p30,e3);
end
for i =573:684
    pos_err(i)=point_to_line_distance(p(i,:),p40,e4);
end
for i =685:810
    pos_err(i)=point_to_line_distance(p(i,:),p50,e5);
end


x=[0 0 0]';
[y1,y2,y3,y4]=fmincon('get_min_ang',x);