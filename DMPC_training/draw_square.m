 function [ output_args ] = draw_square (x,y,r)
%UNTITLED4 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
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
% ��һ��Բ���ϰ���
% x, y: Բ������
% r: Բ�İ뾶
% color: Բ����ɫ����ѡ��Ĭ��Ϊ��ɫ��

    if nargin < 4  % ���û���ṩ��ɫ��Ĭ��ʹ�ú�ɫ
        color = 'k';
    end

    theta = linspace(0, 2*pi, 100); % ���� 0 �� 2�� �� 100 ����
    X = r * cos(theta) + x; % ����Բ�� X ����
    Y = r * sin(theta) + y; % ����Բ�� Y ����
    fill(X, Y, color, 'FaceAlpha', 0.3, 'EdgeColor', color, 'LineWidth', 1.5); % �������Բ
end
