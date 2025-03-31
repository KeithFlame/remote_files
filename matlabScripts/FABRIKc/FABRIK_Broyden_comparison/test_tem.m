% 定义三维三角形的三个顶点
x = [1, 2, 0]; % X 坐标
y = [1, 0, 0]; % Y 坐标
z = [0, 0, 1]; % Z 坐标

% 绘制三维三角形
figure;
fill3(x, y, z, 'magenta'); % 使用紫红色填充
axis equal; % 使坐标轴比例相等
xlabel('X');
ylabel('Y');
zlabel('Z');
title('三维填充三角形');
grid on;