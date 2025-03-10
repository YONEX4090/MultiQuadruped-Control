function [ repulsion ] = compute_repulsion(robot_pose, obs_pose, detect_R)
% 计算机器人受到的避障斥力，适用于圆形障碍物
% obs_pose=[x1 y1 r1; x2 y2 r2; ...]  % 每行表示一个障碍物 (x, y, 半径)

[M,~] = size(obs_pose);
repulsion = [0, 0]; % 初始化 x, y 方向的斥力

for i = 1:M
    ob_x = obs_pose(i,1); % 障碍物 x 坐标
    ob_y = obs_pose(i,2); % 障碍物 y 坐标
    ob_r = obs_pose(i,3); % 障碍物半径
    
    % 计算机器人到障碍物**中心**的距离
    distance_to_center = sqrt((robot_pose(1) - ob_x)^2 + (robot_pose(2) - ob_y)^2);
    
    % 计算机器人到障碍物**边界**的距离
    distance_to_boundary = distance_to_center - ob_r;
    
    % 只有当机器人进入障碍物的影响范围时才施加斥力
    if distance_to_boundary <= detect_R && distance_to_boundary > 0
        temp = 1.0 * (1/distance_to_boundary - 1/detect_R) / (distance_to_boundary^3) * (distance_to_boundary^5);
        
        % 计算斥力方向（指向远离障碍物的方向）
        repulsion(1) = repulsion(1) + temp * ((robot_pose(1) - ob_x) / distance_to_center);
        repulsion(2) = repulsion(2) + temp * ((robot_pose(2) - ob_y) / distance_to_center);
    end
end
end


