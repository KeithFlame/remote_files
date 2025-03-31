% 示例数据
x = linspace(1, 10, 100); % X 轴数据
A = exp(x);               % 正 Y 轴数据（指数增长）
B = exp(x);               % 负 Y 轴数据（指数增长）

% 创建图形窗口
figure;

% 绘制正 Y 轴数据 A
semilogy(x, A, 'b-', 'LineWidth', 2); % 绘制 A 数据
hold on;                             % 保持当前图形

% 绘制负 Y 轴数据 B（取其负值）
semilogy(x, -B, 'r-', 'LineWidth', 2); % 绘制 B 数据的负值

% 设置 Y 轴范围
ylim([-max(B)*1.1, max(A)*1.1]); % 设置 Y 轴范围为负的最大值到正的最大值

% 设置 Y 轴的刻度
yticks([-max(B), 0, max(A)]); % 设置 Y 轴刻度

% 添加图形标题和轴标签
title('Y 轴正区域为数据 A，负区域为数据 B');
xlabel('X 轴');
ylabel('Y 轴');

% 设置图例
legend('数据 A (正)', '数据 B (负)');

% 设置网格
grid on;

% 显示图形
hold off;                           % 释放当前图形