p0=load("F:\code_git\matlabScripts\paper2\data_process\pic_factory\trial_5\refer_tar.log");
position = p0(:,1:3);
r=p0(:,4:7);

%% get tar_position
x = rand(9,1);
x = [4.8971   22.8598   24.7893   50.3531   48.2755  -58.2533   15.7459  122.4479    0.1626]';
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
[y1,y2,y3,y4]=fmincon('get_min_position2',x,[],[],[],[],[],[],[],options);

figure;
hold on; axis equal;
plot3(position(:,1),position(:,2),position(:,3),'r.');

R = eul2rotm(y1(1:3)');
width = y1(4);
height = y1(5);
P1 = y1(6:8);
P2 = P1 - R(:,1)*width;
P3 = P2 + R(:,2)*height;
P4 = P3 + R(:,1)*width;
P_tar = [P1 P2 P3 P4 P1]';
plot3(P_tar(:,1),P_tar(:,2),P_tar(:,3),'b-');
xlabel('x (mm)')
ylabel('y (mm)')
zlabel('z (mm)')

for_opt = position(52:end,:);