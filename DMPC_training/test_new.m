clear;
clc;

% 参数设置
Obstacle = [3; 1];       % 障碍物位置
MaxControl_1 = 0.5;      % 最大控制输入
offset1 = 0.05;          % 偏置

% 定义当前位置范围（只考虑沿 X 方向和 Y 方向变化，构建二维网格）
[X, Y] = meshgrid(-3:0.1:3, -3:0.1:3);
PresentState = [X(:), Y(:)]'; % 将网格数据转换为列向量

% 计算 distance 和初始化 B_x
distance = sqrt((PresentState(1, :) - Obstacle(1)).^2 + (PresentState(2, :) - Obstacle(2)).^2);
B_x = zeros(size(distance));

% 逐点计算 B_x
for i = 1:length(distance)
    % 计算 B_x1
    if distance(i) > MaxControl_1 + offset1
        B_x1 = -log(distance(i) - 0.5) + log(10);
    elseif distance(i) <= MaxControl_1 + offset1
        up = MaxControl_1 + offset1;
        k1 = (up - 0.5)^-2;
        k2 = -1 / (up - 0.5);
        b2 = -log(up - 0.5) + log(10);
        B_x1 = 1/2 * k1 * (distance(i) - up)^2 + k2 * (distance(i) - up) + b2;
    end
    
    % 更新 B_x
    if distance(i) > 10
        B_x(i) = 0;  % distance 大于 10 时 B_x 设置为零
    else
        B_x(i) = B_x1;  % 否则使用计算出的 B_x1
    end
end

% 绘制三维散点图
figure;

% 使用散点图绘制 B_x 与位置的关系
subplot(1, 2, 1);
scatter3(PresentState(1, :), PresentState(2, :), B_x, 10, B_x, 'filled');
colorbar;
title('B_x vs Position');
xlabel('X Position');
ylabel('Y Position');
zlabel('B_x');
grid on;

% 绘制 B_x 与 distance 的关系
subplot(1, 2, 2);
plot(distance, B_x, 'LineWidth', 1.5);
xlabel('Distance');
ylabel('B_x');
title('B_x vs Distance');
grid on;