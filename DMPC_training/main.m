clear all;close all;
load('trained_model.mat');
global tau ConHor_len state_bound Iterations_num num DesiredF State0 E U;
tau=0.05;
Gamma=0.95;   %%  discount factor,  practical design in RL, suitable for long prediction horizon
state_bound=[5;5;5;5]; state_bound1=[2;2;2;2];
Iterations_num=200; %180
MaxTrials=30;
ConHor_len=1; %% Control Horizon length
PreHor_len=10; %% Predictive Horizon length
fol_num=4;        
N=5;             % 4follower and 1 leader
countmax=2000;
dt=0.1;
gama=0.65;
beta=13;
K0=1;
KN=0.2;
goal=[15 15];
m_count = 0;
is_arrive = 0;
% x最高速度m/s],y最高旋转速度[rad/s],x最高加速度[m/ss],y最高加速度[rad/ss]]
Kinematic=[0.7;0.7;0.4;0.4];
attmse(:,1) = [0;0;0;0;0;0];
error_distance = [0;0;0;0];
color='mbgcrkr'; %%%定义颜色标记
type=[2,1,0.5,0.5,2,2];%%%定义线的类型
start_time = clock;
A=[0 0 0 0 1;    
   1 0 0 0 1;
   0 0 0 0 1;
   0 0 1 0 1;
   0 0 0 0 0];
%% Initialization
num=20;
cost=zeros(num,Iterations_num);
Jsum=zeros(num,Iterations_num+1);
for i=1:num
    ANN{i}=CreateANN1(12,2);
    CNN{i}=CreateCNN1(12,12);
end
R_State1=[0;-1;0;1];   %R_State1=[0;1;0;1];

for i=1:num
    if i>=1 && i <=0.5*num
        State{i}=[-2*(i-1)+1*rand(1);-1;0;1];%+2*rand(1)-1
    else
        State{i}=[2*(i-num)+1*rand(1);1;0;1];
    end
    Thita{i}=State{i}(3);
    T{i}=[cos(Thita{i}),sin(Thita{i}),0,0;
        -sin(Thita{i}),cos(Thita{i}),0,0;
        0,0,1,0;
        0,0,0,1];
end

V=diag([1,1,0,0]);V1=diag([0,0,1,1]);

DF=[-2; 0; 0;0];
DesiredF=DF;
for i=2:2*(num-1)
    if i<num-2 ||i>num-2 && i~=num && i<2*(num-1)
        DF(:,i)=-DF(:,i-1);
    elseif i==num-2 || i==2*(num-1)
        DF(:,i)=[DF(2,i-1);DF(1,i-1);0;0];
    elseif i==num
        DF(:,i)=-1*[DF(2,i-1);DF(1,i-1);0;0];
    end
    DesiredF=[DesiredF DF(:,i)];
end

NIError{1}=T{1}*(R_State1-State{1})+T{1}*(State{num}-State{1}+(-DesiredF(:,2*(num-1))));
for i=2:num-1
NIError{i}=T{i}*(V*(State{i-1}-State{i})+DesiredF(:,2*(i-1)-1))+T{i}*(V*(State{i+1}-State{i})+DesiredF(:,2*(i-1)))+V1*(R_State1-State{i});
end
NIError{num}=T{num}*(State{num-1}-State{num}+DesiredF(:,2*(num-1)-1))+T{num}*(State{1}-State{num}+DesiredF(:,2*(num-1)))+V1*(R_State1-State{num});

scale=0.01;
mu=0.001;
Q=1*eye(12)*scale;
R=0.5*eye(2)*scale;
J=[];virtual=[0;0;0;0];
Umax=[1;1];Umin=[-1;-1];
Obstacle=[3;1];OO=zeros(4,4);
P1=[148.133190596417,2.02583848679917,3.51323846649974,46.7730691661438;2.02583848679917,148.133191490445,46.7730693299270,3.51323630780372;3.51323846649974,46.7730693299270,92.6842566525951,8.80444258168160;46.7730691661438,3.51323630780372,8.80444258168160,92.6842564186466];
P2=[148.133190600772,2.02583848790635,3.51323846727139,46.7730691676650;2.02583848790635,148.133191490058,46.7730693308098,3.51323630784338;3.51323846727139,46.7730693308098,92.6842566511360,8.80444258118404;46.7730691676650,3.51323630784338,8.80444258118404,92.6842564014894];
P3=[658.124851396075,11.5177222865393,18.0575925561011,244.738183532405;11.5177222865393,658.124847239358,244.738181009361,18.0575821660109;18.0575925561011,244.738181009361,213.345125351539,21.9218553649452;244.738183532405,18.0575821660109,21.9218553649452,213.345126399689];
P4=[888.158317116642,14.8674148692541,21.2150995166181,285.920616563882;14.8674148692541,888.158303683769,285.920611242892,21.2150929470356;21.2150995166181,285.920611242892,214.761632207292,22.3319105649926;285.920616563882,21.2150929470356,22.3319105649926,214.761634284998];
P5=[888.158317389190,14.8674148497524,21.2150995112752,285.920616599051;14.8674148497524,888.158303732463,285.920611276001,21.2150929405337;21.2150995112752,285.920611276001,214.761632233751,22.3319105640493;285.920616599051,21.2150929405337,22.3319105640493,214.761634279374];
P6=[658.124851366556,11.5177222796616,18.0575925467624,244.738183446075;11.5177222796616,658.124847194266,244.738180976648,18.0575821612168;18.0575925467624,244.738180976648,213.345125331307,21.9218553602577;244.738183446075,18.0575821612168,21.9218553602577,213.345126344712];
P{1}=scale*P1;P{2}=scale*P2;P{3}=scale*P3;P{4}=scale*P4;
P{5}=scale*P5;P{6}=scale*P6;
if num>6
for i=6:num
    P{i}=scale*P6;
end
end
tic
%% Main loop
run=2;
R_State10=R_State1;
State0=State;
NIError0=NIError;
J=zeros(run,Iterations_num);
%% Ö÷Ñ­»·
for iter=1:run
    disp(['run=',num2str(iter)]);
    R_State1=R_State10;
    NIError=NIError0;
    State=State0;
    
    for k=1:ConHor_len:Iterations_num
        disp(['time step=',num2str(k)]);
        Present_rx1=R_State1;
        RealError=NIError;
        RealState=State;
        f=1;
        %% Determine whether the learning is successful in the current prediction time domain
        while(f>=1)
            Present_x1=Present_rx1;
            PresentError=RealError;
            PresentState=RealState;
            Err1=0;Err2=0;
            timesteps=0; j=0;
            while(max(abs(PresentError{1})-state_bound)<=0 && timesteps<PreHor_len)
                for i=1:num
                    c_input{i}=PresentError{i}./state_bound1;
                end
                ANN{1}=NNProcess1(ANN{1},[c_input{1};virtual;c_input{num}]);
                if num>2
                for i=2:num-1
                    ANN{i}=NNProcess1(ANN{i},[c_input{i};c_input{i-1};c_input{i+1}]);
                end
                end
                ANN{num}=NNProcess1(ANN{num},[c_input{num};c_input{1};c_input{num-1}]);
                
                for i=1:num
                    u{i}=ANN{i}.NetworkOut;
                    FutureState{i}=robot(PresentState{i},u{i});
                end
                Future_x1=desired_position(Present_x1);
                
                [MJ{1,1},MJ{1,2},MJ{1,3},MJ{1,4}]=sys_process_four(PresentError{1},u{1},PresentState{1},PresentState{num},PresentState{2});
                if num>2
                for i=2:num-1
                    [MJ{i,1},MJ{i,2},MJ{i,3},MJ{i,4}]=sys_process_four(PresentError{i},u{i},PresentState{i},PresentState{i-1},PresentState{i+1});
                end
                end
                [MJ{num,1},MJ{num,2},MJ{num,3},MJ{num,4}]=sys_process_four(PresentError{num},u{num},PresentState{num},PresentState{num-1},PresentState{1});
                
                for i=1:num
                    Thita{i}=FutureState{i}(3);
                    T{i}=[cos(Thita{i}),sin(Thita{i}),0,0;
                        -sin(Thita{i}),cos(Thita{i}),0,0;
                        0,0,1,0;
                        0,0,0,1];
                end
                
                FutureError{1}=T{1}*(Future_x1-FutureState{1})+T{1}*(FutureState{num}-FutureState{1}+(-DesiredF(:,2*(num-1))));
                if num>2
                for i=2:num-1
                    FutureError{i}=T{i}*(V*(FutureState{i-1}-FutureState{i})+DesiredF(:,2*(i-1)-1))+T{i}*(V*(FutureState{i+1}-FutureState{i})+DesiredF(:,2*(i-1)))+V1*(R_State1-FutureState{i});
                end
                end
                FutureError{num}=T{num}*(FutureState{num-1}-FutureState{num}+DesiredF(:,2*(num-1)-1))+T{num}*(FutureState{1}-FutureState{num}+DesiredF(:,2*(num-1)))+V1*(R_State1-FutureState{num});
                
                for i=1:num
                    f_input{i}=FutureError{i}./state_bound1;
                end
                dR_dZ{1}=2*Q*[PresentError{1};PresentError{2};PresentError{num}];
                if num>2
                for i=2:num-1
                    dR_dZ{i}=2*Q*[PresentError{i};PresentError{i-1};PresentError{i+1}];
                end
                end
                dR_dZ{num}=2*Q*[PresentError{num};PresentError{1};PresentError{num-1}];
                %% CNN output
                CNN{1}=NNProcess1(CNN{1},[f_input{1};f_input{2};f_input{num}]);
                if num>2
                for i=2:num-1
                    CNN{i}=NNProcess1(CNN{i},[f_input{i};f_input{i-1};f_input{i+1}]);
                end
                end
                CNN{num}=NNProcess1(CNN{num},[f_input{num};f_input{1};f_input{num-1};]);
                
                if(timesteps<PreHor_len-1)
                    for i=1:num
                        FutureLambda{i}=CNN{i}.NetworkOut;
                    end
                else
                    for i=1:num
                        FutureLambda{i}=2*[P{i},OO,OO;OO,OO,OO;OO,OO,OO;]*[FutureError{i};[0;0;0;0;];[0;0;0;0;]];
                    end
                end
                CNN{1}=NNProcess1(CNN{1},[c_input{1};c_input{2};c_input{num};]);
                if num>2
                for i=2:num-1
                CNN{i}=NNProcess1(CNN{i},[c_input{i};c_input{i-1};c_input{i+1};]);
                end
                end
                CNN{num}=NNProcess1(CNN{num},[c_input{num};c_input{1};c_input{num-1};]);
                
                for i=1:num
                    PresentLambda{i}=CNN{i}.NetworkOut;
                end
                %% Update ANN and CNN
                ANNError{1}=2*(R)*u{1}+Gamma*(MJ{1,2}'*FutureLambda{1}(1:4)+MJ{1,2}'*FutureLambda{2}(5:8)+MJ{1,2}'*FutureLambda{num}(5:8));                % calculate ANN error signal
                %ANNError{2}=2*(R)*u{2}+mu*dB_u{2}+Gamma*(MJ{2,2}'*FutureLambda{2}(1:4)+MJ{2,2}'*FutureLambda{3}(5:8));
                if num>2
                for i=2:num-1
                    ANNError{i}=2*(R)*u{i}+Gamma*(MJ{i,2}'*FutureLambda{i}(1:4)+MJ{i,2}'*FutureLambda{i-1}(9:12)+MJ{i,2}'*FutureLambda{i+1}(5:8));
                end
                end
                ANNError{num}=2*(R)*u{num}+Gamma*(MJ{num,2}'*FutureLambda{num}(1:4)+MJ{num,2}'*FutureLambda{1}(9:12)+MJ{num,2}'*FutureLambda{num-1}(9:12));
                
                for i=1:num
                    ANN{i}=NNTrain1(ANN{i},ANNError{i});
                end
                Z0=zeros(4,4);
                CNNTarget{1}=dR_dZ{1}+Gamma*[MJ{1,1}',MJ{2,3}',MJ{num,3}';MJ{1,3}',MJ{2,1}',Z0;MJ{1,4}',Z0',MJ{num,1}';]*[FutureLambda{1}(1:4);FutureLambda{2}(5:8);FutureLambda{num}(5:8)];
                if num>2
                for i=2:num-1
                    CNNTarget{i}=dR_dZ{i}+Gamma*[MJ{i,1}',MJ{i-1,4}',MJ{i+1,3}';MJ{i,3}',MJ{i-1,1}',Z0';MJ{i,4}',Z0',MJ{i+1,1}';]*[FutureLambda{i}(1:4);FutureLambda{i-1}(9:12);FutureLambda{i+1}(5:8)];
                end
                end
                i=num;
                CNNTarget{num}=dR_dZ{num}+Gamma*[MJ{num,1}',MJ{1,4}',MJ{num-1,4}';MJ{num,3}',MJ{1,1}',Z0';MJ{num,4}',Z0,MJ{num-1,1}';]*[FutureLambda{i}(1:4);FutureLambda{1}(9:12);FutureLambda{i-1}(9:12)];
                
                for i=1:num
                    CNNError{i}=PresentLambda{i}-CNNTarget{i};
                    CNN{i}=NNTrain1(CNN{i},CNNError{i});
                    PresentState{i}=FutureState{i};
                    PresentError{i}=FutureError{i};
                end
                Present_x1=Future_x1;
                timesteps=timesteps+1;
                if timesteps==2
                    j=j+[PresentError{1};PresentError{2};PresentError{num};]'*Q*[PresentError{1};PresentError{2};PresentError{num};]+u{1}'*R*u{1};
                    if num>2
                    for i=2:num-1
                        j=j+[PresentError{i};PresentError{i-1};PresentError{i+1};]'*Q*[PresentError{i};PresentError{i-1};PresentError{i+1};]+u{i}'*R*u{i};
                    end
                    end
                    j=j+[PresentError{num};PresentError{1};PresentError{num-1};]'*Q*[PresentError{num};PresentError{1};PresentError{num-1};]+u{num}'*R*u{num};
                end
            end
            J(iter,k)=j/scale;
            %% Determine whether learning is successful whether learning is successful
            if(timesteps==PreHor_len)  %% success
                [NIError,State,R_State1]=draw_fig2Scale(ANN,NIError,State,R_State1,Obstacle,k);
                f=0;
            else                        %% fail
                f=f+1;
            end
            %% If the learning fails for MaxTrials times in this time domain, reinitialize the network
            if(f>MaxTrials)
                for i=1:num
                    ANN{i}=CreateANN1(12,2);
                    ANN_u{i}=CreateANN_x(2,2);
                    ANN_x{i}=CreateANN_x(2,2);
                    CNN{i}=CreateCNN1(12,12);
                    CNN_x{1}=CreateCNN_x(2,2);
                end
                f=1;
            end
        end
    end
end
toc
for timesteps=1:Iterations_num
            cost(1,timesteps)=[E{1}(:,timesteps);E{num}(:,timesteps)]'*1*[E{1}(:,timesteps);E{num}(:,timesteps)]+U{1}(:,timesteps)'*0.5*U{1}(:,timesteps);
             Jsum(1,timesteps+1)=Jsum(1,timesteps)+cost(1,timesteps);
             if num>2
            for i=2:num-1
            cost(i,timesteps)=[E{i-1}(:,timesteps);E{i}(:,timesteps);E{i+1}(:,timesteps)]'*1*[E{i-1}(:,timesteps);E{i}(:,timesteps);E{i+1}(:,timesteps)]+U{i}(:,timesteps)'*0.5*U{i}(:,timesteps);
                                    Jsum(i,timesteps+1)=Jsum(i,timesteps)+cost(i,timesteps);
            end
             end
            cost(num,timesteps)=[E{num-1}(:,timesteps);E{num}(:,timesteps);E{1}(:,timesteps)]'*1*[E{num-1}(:,timesteps);E{num}(:,timesteps);E{1}(:,timesteps)]+U{num}(:,timesteps)'*0.5*U{num}(:,timesteps);
            Jsum(num,timesteps+1)=Jsum(num,timesteps)+cost(num,timesteps);
  timesteps=timesteps+1;
end          
function [NIError,State,R_State1]=draw_fig2Scale(ANN,NIError,State,R_State1,Obstacle,k)
global ConHor_len state_bound Iterations_num num DesiredF State0 E U X;
persistent PE Present_x1;
if(k==1)
    for i=1:num
        E{i}=zeros(4,Iterations_num);
        U{i}=zeros(2,Iterations_num);
        X{i}=zeros(4,Iterations_num);
    end
    PresentState=State0;
    R_State1=[0;-1;0;1];
    for i=1:num
        PE{i}=NIError{i};
    end
    Present_x1=[0;-1;0;1];
end

V=diag([1,1,0,0]);V1=diag([0,0,1,1]);virtual=[0;0;0;0];
timesteps=0;
for i=1:num
    PresentError{i}=NIError{i};
    PresentState{i}=State{i};
end
state_bound=[2;2;2;2];
Umax=[1;1];Umin=[-1;-1];

while(timesteps<ConHor_len)
    for i=1:num
        c_input{i}=PresentError{i}./state_bound;
    end
    ANN{1}=NNProcess1(ANN{1},[c_input{1};c_input{2};c_input{num}]);
    for i=2:num-1
        ANN{i}=NNProcess1(ANN{i},[c_input{i};c_input{i-1};c_input{i+1};]);
    end
    ANN{num}=NNProcess1(ANN{num},[c_input{num};c_input{1};c_input{num-1}]);
    for i=1:num
        u{i}=ANN{i}.NetworkOut;
        FutureState{i}=robot(PresentState{i},u{i});
        Thita{i}=FutureState{i}(3);
        T{i}=[cos(Thita{i}),sin(Thita{i}),0,0;
            -sin(Thita{i}),cos(Thita{i}),0,0;
            0,0,1,0;
            0,0,0,1];
    end
    Future_x1=desired_position(Present_x1);
    
    FE{1}=T{1}*(Future_x1-FutureState{1})+T{1}*(FutureState{num}-FutureState{1}+(-DesiredF(:,2*(num-1))));
    for i=2:num-1
        FE{i}=T{i}*(V*(FutureState{i-1}-FutureState{i})+DesiredF(:,2*(i-1)-1))+T{i}*(V*(FutureState{i+1}-FutureState{i})+DesiredF(:,2*(i-1)))+V1*(R_State1-FutureState{i});
    end
    FE{num}=T{num}*(FutureState{num-1}-FutureState{num}+DesiredF(:,2*(num-1)-1))+T{num}*(FutureState{1}-FutureState{num}+DesiredF(:,2*(num-1)))+V1*(R_State1-FutureState{num});
    
    for i=1:num
        FutureError{i}=FE{i};
        E{i}(:,k+timesteps)=PE{i};
        X{i}(:,k+timesteps)=PresentState{i};
        U{i}(:,k+timesteps)=u{i};
        PresentError{i}=FutureError{i};
        PresentState{i}=FutureState{i};
        PE{i}=FE{i};
    end
    Present_x1=Future_x1;
    timesteps=timesteps+1;
end
for i=1:num
    NIError{i}=PE{i};
    State{i}=PresentState{i};
end
R_State1=Present_x1;
end
        init_f=[ 0 3 pi/4;
                 0 2 pi/4; 
                 0 -2 pi/4;
                 0 -1 pi/4;
                 0 0 pi/4];  
    pose_x=init_f(:,1);
    pose_y=init_f(:,2);
    pose_th=init_f(:,3);
    ob_temp=[3 6 0;5 4 0.1; 5 8 0;5 11 0;8 5 0; 6 6 0.1;8 9 0;9 11 0.1; 7 5 0.0;7 8 0;7.6 11 0.0; 6 10 0;10 9 0;8 6.5 0.2;10 9 0;10.5 6 0];
    delta_x=[-1.5 -3 0 0 0];      
    delta_y=[0 0 -1.5 -3 0];  
    V_x(:,1)=[0;0;0;0;0];
    V_y(:,1)=[0;0;0;0;0]; 
    k=0;
    d_max=2;
    detect_R=1;
    ideal_posex=init_f(:,1);
    ideal_posey=init_f(:,2);
    %% Circle
    for count=1:countmax
        k=k+1;
        %%%Move towards the target point
        distance=sqrt((goal(1)-pose_x(N,k))^2+(goal(2)-pose_y(N,k))^2);
        th=atan2(goal(2)-pose_y(N,k),goal(1)-pose_x(N,k));
        if distance>2   
            distance=2;
        end
        V_x(N,k+1)=KN*distance*cos(th); 
        V_y(N,k+1)=KN*distance*sin(th);
        mse_leader=0;
        if(rem(k,5)==1&&k>1)    
            ideal_posex(N,(k-1)/5+1)=V_x(N,k+1)*dt*5+pose_x(N,k);
            ideal_posey(N,(k-1)/5+1)=V_y(N,k+1)*dt*5+pose_y(N,k);
        end
        ob_pose=ob_temp;
        repulsion=compute_repulsion([pose_x(N,k),pose_y(N,k)],ob_pose,detect_R);        
        %%%%%
        V_x(N,k+1)=V_x(N,k+1)+beta*repulsion(1);
        V_y(N,k+1)=V_y(N,k+1)+beta*repulsion(2);
        
        if(distance>1&&abs(V_x(N,k+1))<=0.1&&abs(V_y(N,k+1))<=0.1)
%             V_x(N,k+1)=beta*(1+rand(1))*repulsion(1);
%             V_y(N,k+1)=beta*(1+rand(1))*repulsion(2);
            V_x(N,k+1)=-1+2*rand(1);
            V_y(N,k+1)=-1+2*rand(1);
        end
        att_mse=[]; 
        if(rem(k,5)==1&&k>1)
            attmse(N+1,(k-1)/5)=0;
            for j=1:fol_num
                att_mse(j) = cal_mse([pose_x(j,k),pose_y(j,k)],[ideal_posex(j,(k-1)/5),ideal_posey(j,(k-1)/5)]);
                attmse(j,(k-1)/5) = abs(att_mse(j)-0.2);
                attmse(N+1,(k-1)/5) = attmse(N+1,(k-1)/5) + abs(att_mse(j)-0.2);
            end
        end
        for i=1:fol_num  %fol_num=4      
            sum_delta_x=0;
            sum_delta_y=0;
            for j=1:N 
                sum_delta_x=sum_delta_x+A(i,j)*((pose_x(j,k)-pose_x(i,k))-(delta_x(j)-delta_x(i)));
                sum_delta_y=sum_delta_y+A(i,j)*((pose_y(j,k)-pose_y(i,k))-(delta_y(j)-delta_y(i)));   
            end
%             distance=[];
            error_distance(i,k+1)=sqrt(sum_delta_x^2+ sum_delta_y^2);
            th=atan2(sum_delta_y, sum_delta_x);
%             if error_distance(i,k+1)>d_max
%                 error_distance(i,k+1)=d_max;
%             end
            V_x(i,k+1)=gama*error_distance(i,k+1)*cos(th);
            V_y(i,k+1)=gama*error_distance(i,k+1)*sin(th);
%             disp(['i is',num2str(i)]);%打印distance
%             disp(['distance is',num2str(distance(i,k+1))]);%打印distance
%             disp(['V_x1 is',num2str(V_x(1,k+1))]);
%             disp(['V_y1 is',num2str(V_y(1,k+1))]);
            if(rem(k,5)==1&&k>1)
                ideal_posex(i,(k-1)/5+1)=V_x(i,k+1)*dt*5+pose_x(i,k);
                ideal_posey(i,(k-1)/5+1)=V_y(i,k+1)*dt*5+pose_y(i,k);
            end
            kk=0;
            for j=1:N
                if j~=i
                    kk=kk+1;
                    obs_pose(kk,1)=pose_x(j,k);
                    obs_pose(kk,2)=pose_y(j,k);
                end
            end
            ob_pose=[ob_temp];
            repulsion=compute_repulsion([pose_x(i,k),pose_y(i,k)],ob_pose,detect_R);        
            %%%%%
            V_x(i,k+1)=K0*V_x(N,k)+V_x(i,k+1)+beta*repulsion(1);
            V_y(i,k+1)=K0*V_y(N,k)+V_y(i,k+1)+beta*repulsion(2);
            if(error_distance(i,k+1)>0.5&&abs(V_x(i,k+1))<=0.1&&abs(V_y(i,k+1))<=0.1&&distance>1)
                V_x(i,k+1)=-1+2*rand(1);
                V_y(i,k+1)=-1+2*rand(1);
                disp(['distance is',num2str(error_distance(i,k+1))]);%打印distance
                disp(['rand V_x is',num2str(V_x(i,k+1))]);
                disp(['rand V_y is',num2str(V_y(i,k+1))]);
            end
% %             out=confine([V_x(i,k) V_y(i,k)],[V_x(i,k+1) V_y(i,k+1)],Kinematic);
% %             V_x(i,k+1)=out(1);
% %             V_y(i,k+1)=out(2);
        end
        %%
        for i=1:N
            out=confine([V_x(i,k) V_y(i,k)],[V_x(i,k+1) V_y(i,k+1)],Kinematic,0.1);
%             out=[V_x(i,k+1) V_y(i,k+1)];
            V_x(i,k+1)=out(1);
            V_y(i,k+1)=out(2);
            pose_x(i,k+1)=pose_x(i,k)+dt*V_x(i,k+1);
            pose_y(i,k+1)=pose_y(i,k)+dt*V_y(i,k+1);
            pose_th(i,k+1)=atan2(V_y(i,k+1),V_x(i,k+1));
        end
        if(rem(k,5)==1&&k>1)
            mse_leader = cal_mse([pose_x(N,k),pose_y(N,k)],[ideal_posex(N,(k-1)/5),ideal_posey(N,(k-1)/5)]);
            attmse(N,(k-1)/5)=mse_leader;
        end
        tt_x(1:4,k)=pose_x(5,k);
        error_x(:,k)=tt_x(1:4,k)-pose_x(1:4,k)+(delta_x(1:4))';
        tt_y(1:4,k)=pose_y(5,k);
        error_y(:,k)=tt_y(1:4,k)-pose_y(1:4,k)+(delta_y(1:4))';
        if(k==100)
            bbb=1;
        end
        %% ====Animation====
        area = compute_area(pose_x(N,k+1),pose_y(N,k+1),10);
        hold off;
        ArrowLength=0.7;% 
        for j=1:N
            quiver(pose_x(j,k+1),pose_y(j,k+1),ArrowLength*cos(pose_th(j,k+1)),ArrowLength*sin(pose_th(j,k+1)),'.','color',color(1,j),'LineWidth',1.3);hold on;
            % quiver(pose_x(j,k+1), pose_y(j,k+1), ArrowLength*cos(pose_th(j,k+1)), ArrowLength*sin(pose_th(j,k+1)), '.', 'color', 'k', 'LineWidth', 1.3);
            draw_circle(pose_x(j,k+1),pose_y(j,k+1),0.1,j);hold on;
        end
        obn = size(ob_temp);
        for i =1:obn
            draw_square(ob_temp(i,1),ob_temp(i,2),0.3);hold on;
        end
        xlabel('X Position(m)');
        ylabel('Y Position(m)');
        axis(area);
        grid on;
        drawnow;    
        %% Determine the termination condition
        now=[pose_x(N,k+1),pose_y(N,k+1)];
        if norm(now-goal)<0.2
            is_arrive = 1;
            end_time = clock;
            disp('Arrive Goal!!');break;
        end
    end
    attmse(:,100)=[0;0;0;0;0;0];
    for i=1:5
        dmax(i)=max(attmse(i,1:99));
    end
    for i=1:5
        if(dmax(i)>0.1)
            attmse(i,1:99)=normalization(attmse(i,1:99),0,dmax(i),0,1);
        else
            attmse(i,1:99)=normalization(attmse(i,1:99),0,max(dmax(:)),0,1);
        end
    end
    save('attmse.mat','attmse');
%     label=svm(1)   
%     load('data.mat')
%     b=attmse(1:5,:);
%     a=[a;b];
%     save('data.mat','a');

%     xlswrite('attmse.xlsx',attmse);
    %% Drawing
    figure                               
    for i=1:N
        plot(pose_x(i,:),pose_y(i,:),color(1,i),'LineWidth',3);
        hold on
    end
    for i=1:N
        plot(pose_x(i,1),pose_y(i,1),'p','color',color(1,i),'LineWidth',3);
        hold on
    end
    for i=1:N
        plot(pose_x(i,k),pose_y(i,k),'h','color',color(1,i),'LineWidth',3);
        hold on
    end
    for i =1:obn
         draw_square(ob_temp(i,1),ob_temp(i,2),0.3);hold on;
    end
%     plot(ob_temp(:,1),ob_temp(:,2),'Xk','LineWidth',2);hold on;
    grid on;
    % fill(x1,y1,'k')             
    % fill(x2,y2,'k')             
    xlabel('x');
    ylabel('y');
    legend('follower1','follower2','follower3','follower4','leader','Location','NorthWest', 'FontSize', 16);
    xlabel('X Position (m)', 'FontSize', 16);
    ylabel('Y Position (m)', 'FontSize', 16);
    %% Draw error graph
    cost_time = 3600*(end_time(4)-start_time(4)) + 60 * (end_time(5)-start_time(5)) + (end_time(6) - start_time(6));
    kx=cost_time/k;
    cx=0:kx:cost_time;
    figure                                
    error=sqrt(error_x.^2+error_y.^2);
    for i=1:4
        plot(cx(1:k-1),error(i,1:k-1),color(1,i),'LineWidth',1.5);
        hold on;
    end
    legend('follower1','follower2','follower3','follower4', 'FontSize', 16);
    xlabel('T(s)', 'FontSize', 16);
    ylabel('Formation Tracking error (m)', 'FontSize', 16);

function [ next] = confine(current,next,Kinematic,dt)
%% Limit V_x
delta_x=next(1)-current(1);
if delta_x>=0
    next(1)=min(current(1)+delta_x,current(1)+Kinematic(3)*dt);
else
    next(1)=max(current(1)+delta_x,current(1)-Kinematic(3)*dt);
end
if next(1)>=0
    next(1)=min(next(1),Kinematic(1));
else
    next(1)=max(next(1),-Kinematic(1));
end
%% Limit V_y
delta_y=next(2)-current(2);
if delta_y>=0
    next(2)=min(current(2)+delta_y,current(2)+Kinematic(4)*dt);
else
    next(2)=max(current(2)+delta_y,current(2)-Kinematic(4)*dt);
end
if next(2)>=0
    next(2)=min(next(2),Kinematic(2));
else
    next(2)=max(next(2),-Kinematic(2));
end
end

function NN=NNTrain1(NN,NNError)
Delta2=NNError;
dB1=Delta2;
dW1=Delta2*tanh(NN.NetworkIn)';
NN.W1=NN.W1-NN.LR*dW1;
NN.B1=NN.B1-NN.LR*dB1;
end
function ANN=CreateANN1(x_dim,u_dim)
ANN.HiddenUnitNum=5;  
ANN.InDim=x_dim;        
ANN.OutDim=u_dim;          
ANN.LR=0.2;            
ANN.W1=1*rand(ANN.OutDim,ANN.InDim)-0.5;  
ANN.B1=1*rand(ANN.OutDim,1)-0.5;  
end
function CNN=CreateCNN1(x_dim,y_dim)
CNN.HiddenUnitNum=5;   
CNN.InDim=x_dim;          
CNN.OutDim=y_dim;          
CNN.LR=0.4;             

CNN.W1=1*rand(CNN.OutDim,CNN.InDim)-0.5;  
CNN.B1=1*rand(CNN.OutDim,1)-0.5;     
end
function NN=NNProcess1(NN,input)
NN.NetworkIn=input;
NN.NetworkOut=NN.W1*tanh(NN.NetworkIn)+NN.B1;
end

function Future_x=desired_position(x)
global tau;
vr=1;
wr=0;
FutureState(1,1)=x(1)+tau*vr*cos(x(3));
FutureState(2,1)=x(2)+tau*vr*sin(x(3));
FutureState(3,1)=x(3)+tau*wr;
FutureState(4,1)=vr;
if(FutureState(3,1)>pi)
    FutureState(3,1)=FutureState(3,1)-2*pi;
elseif(FutureState(3,1)<-pi)
    FutureState(3,1)=FutureState(3,1)+2*pi;
end
Future_x=[FutureState(1,1);FutureState(2,1);FutureState(3,1);FutureState(4,1)];
end
function FutureState=robot(x,u)
global tau;
v=x(4)+tau*u(1);
FutureState(1,1)=x(1)+tau*v*cos(x(3));
FutureState(2,1)=x(2)+tau*v*sin(x(3));
FutureState(3,1)=x(3)+tau*u(2);
FutureState(4,1)=v;
if(FutureState(3,1)>pi)
    FutureState(3,1)=FutureState(3,1)-2*pi;
elseif(FutureState(3,1)<-pi)
    FutureState(3,1)=FutureState(3,1)+2*pi;
end
end
function [MSJ1,MCJ1,MSJ12,MSJ14]=sys_process_four(ep,u,PresentState1,PresentState2,PresentState4)
global tau;
ep1=ep(1);
ep2=ep(2);
ep3=ep(3);
ep4=ep(4);
u1=u(1);
u2=u(2);
wr=0;
vr=1;
v4=PresentState4(4);
v2=PresentState2(4);
thita2=PresentState2(3);
thita1=PresentState1(3);
thita4=PresentState4(3);
% 
MSJ1 = [1, tau*u2, -tau*sin(ep3)*vr-tau*sin(thita2-thita1)*(v2)-tau*sin(thita4-thita1)*(v4),2*tau;
        -tau*u2, 1, tau*cos(ep3)*vr+tau*cos(thita2-thita1)*(v2)+tau*cos(thita4-thita1)*(v4),0;
        0,      0,    1,0;
        0,0,0,1];
MCJ1 =[ 0,  ep2*tau;
    0, -ep1*tau;
    0,     -tau;
    -tau,0];
MSJ12 = [0, 0, tau*sin(thita2-thita1)*(v2),-tau*cos(thita2-thita1);
        0, 0, -tau*cos(thita2-thita1)*(v2),-tau*sin(thita2-thita1);
        0,0, 0,0;
        0,0,0,0];
MSJ14 = [0, 0, tau*sin(thita4-thita1)*(v4),-tau*cos(thita4-thita1);
        0, 0, -tau*cos(thita4-thita1)*(v4),-tau*sin(thita4-thita1);
        0,0, 0,0;
        0,0,0,0];
end