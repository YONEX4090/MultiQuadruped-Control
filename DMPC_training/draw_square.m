 function [ output_args ] = draw_square (x,y,r)
%UNTITLED4 此处显示有关此函数的摘要
%   此处显示详细说明
% if(nargin==4)
%     color='-k';
% end
% if (n == 5)
%     color ='r';
% else
%     color ='-k';
% end
% x1 = [x-r,x+r,x+r,x-r];
% y1 = [y+r,y+r,y-r,y-r];
% fill(x1,y1,'k')
% end

% function draw_circle(x, y, r, color)
% 画一个圆形障碍物
% x, y: 圆心坐标
% r: 圆的半径
% color: 圆的颜色（可选，默认为黑色）

    if nargin < 4  % 如果没有提供颜色，默认使用黑色
        color = 'k';
    end

    theta = linspace(0, 2*pi, 100); % 生成 0 到 2π 的 100 个点
    X = r * cos(theta) + x; % 计算圆的 X 坐标
    Y = r * sin(theta) + y; % 计算圆的 Y 坐标
    fill(X, Y, color, 'FaceAlpha', 0.3, 'EdgeColor', color, 'LineWidth', 1.5); % 画出填充圆
end
