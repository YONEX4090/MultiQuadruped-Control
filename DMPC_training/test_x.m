clear;
clc;

% 参数设置
Obstacle = [3; 1];       % 障碍物位置
MaxControl_1 = 1;        % 最大控制输入1
MaxControl_2 = 1;        % 最大控制输入2
g_1max = 1 / MaxControl_1;
g_2max = 1 / MaxControl_2;

offset1 = 0.1;
offset2 = 0.1;

% 定义当前位置范围（只考虑沿 X 方向和 Y 方向变化，构建二维网格）
[X, Y] = meshgrid(-3:0.1:3, -3:0.1:3);
PresentState = [X(:), Y(:)]'; % 将网格数据转换为列向量

% 计算 distance 和初始化 B_x1, B_x2
distance = sqrt((PresentState(1, :) - Obstacle(1)).^2 + (PresentState(2, :) - Obstacle(2)).^2);
B_x1 = zeros(size(distance));
B_x2 = zeros(size(distance));

% 逐点计算 B_x1 和 B_x2
for i = 1:length(distance)
    % 计算 B_x1
    if distance(i) > MaxControl_1 + offset1
        B_x1(i) = (g_1max / (1 - g_1max * distance(i))) * abs(PresentState(1, i) - Obstacle(1)) / distance(i);
    else
        up1 = MaxControl_1 + offset1;
        k1 = g_1max * (-1) * (up1 - g_1max * up1^2)^(-2) * (1 - 2 * g_1max * up1) * abs(PresentState(1, i) - Obstacle(1));
        b1 = (g_1max / (1 - g_1max * up1)) * abs(PresentState(1, i) - Obstacle(1)) / up1;
        B_x1(i) = k1 * (distance(i) - up1) + b1;
    end
    
    % 计算 B_x2
    if distance(i) > MaxControl_2 + offset2
        B_x2(i) = (g_2max / (1 - g_2max * distance(i))) * abs(PresentState(2, i) - Obstacle(2)) / distance(i);
    else
        up2 = MaxControl_2 + offset2;
        k2 = g_2max * (-1) * (up2 - g_2max * up2^2)^(-2) * (1 - 2 * g_2max * up2) * abs(PresentState(2, i) - Obstacle(2));
        b2 = (g_2max / (1 - g_2max * up2)) * abs(PresentState(2, i) - Obstacle(2)) / up2;
        B_x2(i) = k2 * (distance(i) - up2) + b2;
    end
end

% 合并结果
B_x = [B_x1; B_x2];

% 绘制结果
figure;

% 绘制 B_x1 的形状
subplot(1, 2, 1);
scatter3(PresentState(1, :), PresentState(2, :), B_x1, 10, B_x1, 'filled');
colorbar;
title('B_x1 Shape');
xlabel('X Position');
ylabel('Y Position');
zlabel('B_x1');
grid on;

% 绘制 B_x2 的形状
subplot(1, 2, 2);
scatter3(PresentState(1, :), PresentState(2, :), B_x2, 10, B_x2, 'filled');
colorbar;
title('B_x2 Shape');
xlabel('X Position');
ylabel('Y Position');
zlabel('B_x2');
grid on;