function [B_x]=B_new(PresentState,Obstacle)
%     global Data
    distance = sqrt((PresentState(1)-Obstacle(1))^2+(PresentState(2)-Obstacle(2))^2);
    MaxControl_1=0.5;
    MaxControl_2=0.5;
    offset1=0.05; %偏置
    offset2=0.05;
%%
    B_x=-log(distance-1)+log(10);
    if (distance>MaxControl_1+offset1)
        B_x1=-log(distance-0.5)+log(10);
    elseif (distance<=MaxControl_1+offset1)
        up=MaxControl_1+offset1;
        k1=(up-0.5)^-2;
        k2=-1/(up-0.5);
        b2=-log(up-0.5)+log(10);
        B_x1=1/2*k1*(distance-up)^2+k2*(distance-up)+b2;
    end
    if (distance>MaxControl_2+offset2)
        B_x2=-log(distance-0.5)+log(10);
    elseif (distance<=MaxControl_1+offset2)
        up=MaxControl_2+offset2;
        k1=(up-0.5)^-2;
        k2=-1/(up-0.5);
        b2=-log(up-0.5)+log(10);
        B_x2=1/2*k1*(distance-up)^2+k2*(distance-up)+b2;
    end
    B_x=[B_x1;B_x2];

if distance>10
     B_x=0;
end
end