function [ repulsion ] = compute_repulsion(robot_pose, obs_pose, detect_R)
% ����������ܵ��ı��ϳ�����������Բ���ϰ���
% obs_pose=[x1 y1 r1; x2 y2 r2; ...]  % ÿ�б�ʾһ���ϰ��� (x, y, �뾶)

[M,~] = size(obs_pose);
repulsion = [0, 0]; % ��ʼ�� x, y ����ĳ���

for i = 1:M
    ob_x = obs_pose(i,1); % �ϰ��� x ����
    ob_y = obs_pose(i,2); % �ϰ��� y ����
    ob_r = obs_pose(i,3); % �ϰ���뾶
    
    % ��������˵��ϰ���**����**�ľ���
    distance_to_center = sqrt((robot_pose(1) - ob_x)^2 + (robot_pose(2) - ob_y)^2);
    
    % ��������˵��ϰ���**�߽�**�ľ���
    distance_to_boundary = distance_to_center - ob_r;
    
    % ֻ�е������˽����ϰ����Ӱ�췶Χʱ��ʩ�ӳ���
    if distance_to_boundary <= detect_R && distance_to_boundary > 0
        temp = 1.0 * (1/distance_to_boundary - 1/detect_R) / (distance_to_boundary^3) * (distance_to_boundary^5);
        
        % �����������ָ��Զ���ϰ���ķ���
        repulsion(1) = repulsion(1) + temp * ((robot_pose(1) - ob_x) / distance_to_center);
        repulsion(2) = repulsion(2) + temp * ((robot_pose(2) - ob_y) / distance_to_center);
    end
end
end


