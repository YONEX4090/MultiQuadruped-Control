clear all;
clc;

% 参数设置
Umaxh = 5;
Umaxl = -5;
g_1max = 1 / Umaxh;
g_1min = 1 / Umaxl;
offset1 = 0.1;

% 初始化变量
U_range = -8:0.1:8; % 定义 U 的范围
B_u1 = zeros(size(U_range)); % 初始化 B_u1 数组

% 计算 B_u1
for idx = 1:length(U_range)
    U = U_range(idx);
    if (U > Umaxl + offset1) && (U < Umaxh - offset1)
        B_u1(idx) = g_1max / (1 - g_1max * U) + g_1min / (1 - g_1min * U) - g_1max - g_1min;
    elseif U >= Umaxh - offset1
        up = Umaxh - offset1;
        k = (g_1max / (1 - g_1max * up))^2 + (g_1min / (1 - g_1min * up))^2;
        b = g_1max / (1 - g_1max * up) + g_1min / (1 - g_1min * up) - g_1max - g_1min;
        B_u1(idx) = k * (U - up) + b;
    elseif U <= Umaxl + offset1
        down = Umaxl + offset1;
        k = (g_1max / (1 - g_1max * down))^2 + (g_1min / (1 - g_1min * down))^2;
        b = g_1max / (1 - g_1max * down) + g_1min / (1 - g_1min * down) - g_1max - g_1min;
        B_u1(idx) = k * (U - down) + b;
    end
end

% 绘制曲线
figure;
plot(U_range, B_u1, 'LineWidth', 1.5);
grid on;
xlabel('U');
ylabel('B_{u1}');
title('B_{u1} vs U');