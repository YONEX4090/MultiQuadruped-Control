clear
close all
clc
import casadi.*

t0 = 0; % start time
tf = 9; % end time
dt = 0.05; % step size
k = t0:dt:tf-dt;

% NMPC finite horizon lengths
M = 10; % formation auxiliary system horizon length
N = 10; % trajectory following horizon length
T = length(k) + max(M, N);
t = t0:dt:(T-1)*dt;

U_0 = 10; % Source Voltage
R = 0.5; % Armature Resistance
R_z = 2e-4; % Internal Resistance of voltage source
J_m = 1e-6; % Inertia of Motor Shaft
L = 100e-3; % Armature Inductance
N_g = 1; % Gearbox Transmission Ratio
l_L = 0.125; % Distance between left wheel and p_obs of wheel axle
l_R = l_L; % Distance between right wheel and p_obs of wheel axle
r = 0.05; % wheel d_safe
r_G = r/N_g;
m_p = 4.385;
m_w = 0.0575;
m = m_p + 2*m_w; % Mass of robot
l_T = 0.11; % distance between p_obs of wheel shaft and p_obs of gravity
J_zp = 0.05; % Intertia of the platform about the centroid
J_zw = 0.002; % Intertia of the wheels about the centroid
J_B = J_zp + 2*J_zw + m_w*(2*l_T^2 + l_L^2 + l_R^2); % Intertia about the p_obs of rotation
K_t = 5.305e-2; % Torque Constant
K_e = K_t; % Torque Constant
k_r = 4e-5; % Damping coefficient of Motor Shaft
k_v = 0.25; % Damping coefficient of linear motion
k_w = 0.15; % Dampin coefficient of rotational motion

alpha = (r_G^2)/(l_L + l_R);
scaled_radius = (alpha/r_G);
a_L = k_r + k_v*l_R*alpha;
a_R = k_r + k_v*l_L*alpha;
c_L = k_r*l_L + k_w*alpha;
c_R = k_r*l_R + k_w*alpha;

b_L = J_m + m*l_R*alpha;
b_R = J_m + m*l_L*alpha;
d_L = J_m*l_L + J_B*alpha;
d_R = J_m*l_R + J_B*alpha;

RL = -(1/L)*[R+R_z R_z;
             R_z R+R_z];
KL = -(K_e/L)*eye(2);
ac_mat = [a_L a_R;
          -c_L c_R];
bd_mat = [d_R -b_R;
          d_L b_L];
LR = [1 1;
      -l_L l_R];
beta = 1/det(bd_mat);

%%% Matrix Definition for low-level controller
A = [RL KL;
     (K_t*beta)*bd_mat*LR -beta*bd_mat*ac_mat];
B = (U_0/L)*eye(4, 2);
C = r_G*[zeros(2) (LR'\eye(2))];
electr_sys = ss(A, B, C, zeros(2));

%%% System Parameters
x = MX.sym('x');
y = MX.sym('y');
theta = MX.sym('theta');

omega = MX.sym('omega');
vel = MX.sym('vel', 2);
phi = MX.sym('phi', 2);
tau = MX.sym('tau', 2);

dim_vel = length(phi);
dim_tau = length(tau);

%%% Robot Kinematics part 1
q = [x; y; theta]; % states
dim_state = length(q);

R_q = [cos(q(3)) -l_T*sin(q(3));
       sin(q(3)) l_T*cos(q(3));
       0 1];

R_q_func = Function('R_q_func', {q}, {R_q});
dq = R_q*vel;
dq_func = Function('dq_func', {q, vel}, {dq});

% Nonlinear solver. Produces q(k + 1) given q(k) and u
q_ode = struct('x', q, 'p', vel, 'ode', dq);
q_solver = integrator('q_solver', 'cvodes', q_ode, t0, dt);

%%% Robot Kinematics part 2
SR_q = scaled_radius*[l_R*cos(q(3)), l_L*cos(q(3));
                      l_R*sin(q(3)), l_L*sin(q(3));
                                  1,            -1];
SD_q = scaled_radius*[-l_T*sin(q(3)),  l_T*sin(q(3));
                       l_T*cos(q(3)), -l_T*cos(q(3));
                                   0,              0];
S_q = SR_q + SD_q;
S_y_q = S_q(1:2, 1:2);
skew_mat = [0 -1;
            1  0];
% angular velocity
omega_q = scaled_radius*(phi(1) - phi(2));
% skew symmetric matrix S(w)
S_w = omega_q*skew_mat;
S_z_w = S_w(1:2, 1:2);
dS_y_q = S_w*S_y_q;
S_w_func = Function('S_w_func', {phi}, {S_w});

%%% Defintion of higher dimension system matrices
% M_q = diag([m; m; J_B]);
% C_q = zeros(3, 1);
% B_q = (N_g/r_G)*[cos(q(3)), cos(q(3));
%                  sin(q(3)), sin(q(3));
%                  l_R      , -l_L];

bar_M_q = m*(scaled_radius^2)*[(l_R^2 + l_T^2), (l_L*l_R - l_T^2);
                             (l_R*l_L - l_T^2), (l_L^2 + l_T^2)]...
          + J_B*(scaled_radius^2)*[1, -1; -1, 1];
bar_C_q_coeff = m*(scaled_radius^2)*(l_R + l_L)*(l_T);
bar_C_q = bar_C_q_coeff*(S_w');
bar_B_q_coeff = N_g;
bar_B_q = bar_B_q_coeff*eye(dim_tau);

dphi = bar_M_q\(-bar_C_q*phi + bar_B_q*tau);
dphi_func = Function('dphi_func', {phi, tau}, {dphi});

x_state = [q; phi];
dx_state = [S_q*phi; dphi];

% Nonlinear solver. Produces x_state(k + 1) given x_state(k) and tau
state_ode = struct('x', x_state, 'p', tau, 'ode', dx_state);
state_solver = integrator('state_solver', 'cvodes', state_ode, t0, dt);
dx_state_func = Function('dx_state_func', {x_state, tau}, {dx_state});

%%% Define Saturation limits
volts_ub = U_0; % Volts
volts_lb = -U_0; % Volts
forward_vel = -C*(A\B)*[volts_ub; volts_ub];
v_ub = forward_vel(1);
v_lb = -v_ub;

traj_constr_ub = [v_ub; v_ub];
traj_constr_lb = [v_lb; v_lb];
max_torque = 0.25;
torque_ub = max_torque*ones(2, 1);
torque_lb = -max_torque*ones(2, 1);

%%% trajectory error variables
y_d = MX.sym('y_d', dim_state-1);
dy_d = MX.sym('dy_d', dim_state-1);
ddy_d = MX.sym('ddy_d', dim_state-1);
eta_in = MX.sym('eta_in', dim_state-1);
aux_in = MX.sym('aux_in', dim_tau);


%%% trajectory error matrices
R_theta = [cos(q(3)), -sin(q(3));
           sin(q(3)),  cos(q(3))];
R_theta_func = Function('R_theta_func', {q}, {R_theta});

calculate_torque = bar_B_q\(bar_M_q*aux_in + bar_C_q*phi);
calculate_torque_func = Function('calculate_torque_func', {x_state, aux_in}, {calculate_torque});

cartesian_acceleration = dS_y_q*phi + S_y_q*dphi;
cartesian_acceleration_func = Function('cartesian_acceleration_func', {phi, theta, tau}, {cartesian_acceleration});

tracking_error = (R_theta')*(y_d - q(1:2));
tracking_error_rate = -S_w*tracking_error + (R_theta')*(dy_d - S_y_q*phi);
aux_state = [tracking_error; tracking_error_rate];
aux_state_func = Function('aux_state_func', {x_state, y_d, dy_d}, {aux_state});

transform_mat = -((R_theta')*S_y_q + scaled_radius*skew_mat*tracking_error*[1 -1]);
tracking_error_accel = ([(S_w')*S_w, -2*S_w]*aux_state...
                        + (R_theta')*(ddy_d - dS_y_q*phi))...
                        + transform_mat*aux_in;
aux_state_rate = [tracking_error_rate; tracking_error_accel];
aux_state_rate_func = Function('aux_state_rate_func',...
                                {x_state, y_d, dy_d, ddy_d, aux_in}, ...
                                {aux_state_rate});

inv_transform_mat = pinv(transform_mat);
convert_eta_to_v = transform_mat\(- [(S_w')*S_w, -2*S_w]*aux_state...
                                  - (R_theta')*(ddy_d - dS_y_q*phi)...
                                  + eta_in);
convert_eta_to_v_func = Function('convert_eta_to_v_func',...
                                    {x_state, y_d, dy_d, ddy_d, eta_in}, {convert_eta_to_v});

%%% Convert cartesian velocities to polar velocities
obtain_velocities = (1/l_T)*[l_T*cos(theta) l_T*sin(theta);
                             -sin(theta) cos(theta)];
obtain_vel_func = Function('obtain_vel_func', {theta}, {obtain_velocities});

%%% Define Communication Network
mrs_size = 2; % number of robots
% adjacency matrix
adjacency = [0 1;
             1 0];
silent_adjacency = zeros(mrs_size);
% leader neighborhood
leader_neighbour = [1 1]; % neighbor-indicator vector
silent_leader_neighbour = ones(1, mrs_size);

%%% formation parameters
Delta = [0,  0;
         0, -1];

%%% leader trajectory (virtual signal generation)
start = [0; 1; 0];
p_l = start;
dp_l = [1; 0; 0];

% Let A be the point at the centre of the shaft connecting the two driven
% wheels.
%%% define velocities of virtual leader over trajectory
v_l = zeros(2, T);
% linear velocity of A relative ot fixed centre
v_l(1, :) = 1*ones(1, T);
% angular velocity of A about point remote centre
v_l(2, :) =  zeros(1, T);

q_l = start; % iniital state of virtual robot
for l=2:T
    q_l_solution = q_solver('x0', q_l, 'p', v_l(:, l-1));
    q_l = full(q_l_solution.xf);
    dq_l = R_q_func(q_l)*v_l(:, l-1);
    p_l(:, l) = q_l;
    dp_l(:, l) = full(dq_l);
end

%%% initial conditions
dim_state = length(q); % dimension of state vector
q_k = zeros(dim_state, mrs_size); % empty vector for states

% Initial Robot possitions and orientations
p1_0 = [0; 1; 0];
p2_0 = [0; 0; 0];

q_next_k = [p1_0 p2_0]; % initial system state
q_next_virtual = q_next_k; % initialise virtual followers
pi_k = q_next_k; % initial virtual follower states

% intialise velocities of virtual followers
virtual_dq_k = [1, 1;
                0, 0;
                0, 0];

%%% position of obstacle
tolerance = 0;
obstacle_radius = 0.1;
d_safe = obstacle_radius + tolerance;
p_obs = [3; 1];

%%% Definition of Discrete-time Control Barrier Function
% extract position information of robot
pos = MX.sym('pos', dim_state-1);
pos_next = MX.sym('delta_pos', dim_state-1);
gamma_CBF = MX.sym('gamma_CBF'); 
C_CBF = [eye(2) zeros(2, 3)];

% larger gamma values corresponds with moving closer to obstacle.
% smaller gamma values corresponds with avoiding the obstacle earlier.
% 0 < gamma_val <= 1
min_gamma = 1e-9;
max_gamma = 1 - min_gamma;
desired_gamma_CBF = 0.2;
desired_gamma_CBF2 = 0.75;

% vector from obstacle position to robot position
r_vec = (pos - p_obs);
r_vec_next = (pos_next - p_obs);

% definitions of CBFs
h_p = (norm(r_vec) - d_safe);
h_p_next = (norm(r_vec_next) - d_safe);

% definitions of CBF constraints (CBF interpolation)
num_interp = 5;
cbf_constraints = MX.zeros(num_interp, 1);
h_p_interp_old = h_p;
for i = 1:num_interp
    alpha = i / num_interp;
    pos_interp = (1 - alpha) * pos + alpha * pos_next;
    
    % Compute CBF for the interpolated point
    r_vec_interp = pos_interp - p_obs;
    h_p_interp = norm(r_vec_interp) - d_safe;
    
    % Apply the CBF constraint at each interpolated point
    cbf_constraints(i) = h_p_interp + (gamma_CBF - 1) * h_p_interp_old;
    h_p_interp_old = h_p_interp;
end
cbf_constr_func = Function('cbf_constr_func', {pos, pos_next, gamma_CBF}, {cbf_constraints});

%%% cost weighting matrices (tune)
% Formation Controller
Q_e = diag([45, 15]);
R_e = eye(dim_state-1);
penalise_vel = 250;
S_e = 50*eye(dim_state-1);
H_e = zeros(dim_state-1);
G_e = eye(dim_state-1);
[P_e,~,~] = icare(H_e, G_e, Q_e, R_e, [], [], []);

% discrete formation error model
formation_sys = ss(H_e, G_e, [], []);
formation_sysd =  c2d(formation_sys, dt);
dt_H_e = formation_sysd.A;
dt_G_e = formation_sysd.B;

% Dynamics Controller
Q_0 = 5*eye(2*(dim_state-1));
R_0 = 2.5*eye(dim_state-1);
R_horizon = sparse(kron(eye(N), R_0));
H_0 = [zeros(dim_state-1) eye(dim_state-1);
       zeros((dim_state-1), 2*(dim_state-1))];
G_0 = [zeros(dim_state-1); eye(dim_state-1)];
[P_0,~,~] = icare(H_0, G_0, Q_0, R_0, [], [], []);

% discrete dynamics error model
dynamics_sys = ss(H_0, G_0, [], []);
dynamics_sysd =  c2d(dynamics_sys, dt);
dt_H_0 = dynamics_sysd.A;
dt_G_0 = dynamics_sysd.B;

%%% Define velocity transformation matrices
vel_forward = scaled_radius*[l_L, l_R;
                               1, -1];

%%% memory allocation
formation_sys_opt_output = repmat([repmat(zeros(dim_state-1, 1), M, 1)], 1, mrs_size);

dynamics_sys_opt_output = repmat([repmat(zeros(dim_state-1, 1), N, 1);...
                    desired_gamma_CBF2], 1, mrs_size);
v_i = repmat(zeros(dim_vel, 1), 1, mrs_size);
wheel_vel_i = repmat(zeros(dim_vel, 1), 1, mrs_size);
tau_i = repmat(zeros(dim_tau, 1), 1, mrs_size);

desired_y_rate_history = repmat(zeros(dim_state-1, 1), 1, mrs_size);
desired_y_accel_history = repmat(zeros(dim_state-1, 1), 1, mrs_size);

%%% defining local robot formation relationship
Delta_ij = cell(mrs_size);
for i = 1:mrs_size
    for j = 1:mrs_size
        if (i ~= j)
            Delta_ij{i, j} = Delta(:, i) - Delta(:, j);
        else
            Delta_ij{i, j} = zeros(length(Delta(:, 1)), 1);
        end
    end
end

formation_sys_opt_params = MX.sym('formation_sys_opt_params', length(formation_sys_opt_output));
dynamics_sys_opt_pars = MX.sym('dynamics_sys_opt_pars', length(dynamics_sys_opt_output));

q_i_library = repmat(zeros(dim_state, length(k)), 1, 1, mrs_size);
formation_error = repmat(zeros(dim_state-1, length(k)), 1, 1, mrs_size);
vel_library = repmat(zeros(dim_vel, length(k)), 1, 1, mrs_size);
tau_library = repmat(zeros(dim_tau, length(k)), 1, 1, mrs_size);
wheeled_vel_library = repmat(zeros(dim_vel, length(k)), 1, 1, mrs_size);

lmin_qe = min(eig(Q_e));
lmin_q0 = min(eig(Q_0));

gamma = min(eig(Q_e))/max(eig(P_e));
gamma_q = min(eig(Q_0))/max(eig(P_0));

formation_beta_val = 80;
dyn_beta_val = 50;

%%% Define IPOPT options
opts = struct;
opts.ipopt.tol = 1e-8; % Tolerance for optimization
opts.ipopt.constr_viol_tol = 1e-8; % Constraint violation tolerance
opts.ipopt.acceptable_tol = 1e-6;
opts.ipopt.acceptable_constr_viol_tol = 1e-6;
opts.ipopt.max_iter = 120;  % Max number of iterations
opts.ipopt.print_level = 0; % Suppress output (0 for no output)
opts.print_time = false;
opts.ipopt.sb = 'yes'; % Suppress the IPOPT startup banner

%%% optimisation parameter bounds
form_err_lb = [repmat(-inf*ones(dim_state-1, 1), M, 1)];
form_err_ub = [repmat(inf*ones(dim_state-1, 1), M, 1)];
form_err_lbx = form_err_lb;
form_err_ubx = form_err_ub;
form_err_lbg = [zeros(num_interp, 1); traj_constr_lb];
form_err_ubg = [inf*ones(num_interp, 1); traj_constr_ub];

dynamics_lb = [repmat(-inf*ones(dim_state-1, 1), N, 1);...
                    min_gamma];
dynamics_ub = [repmat(inf*ones(dim_state-1, 1), N, 1);...
                    max_gamma];
dynamics_lbx = dynamics_lb;
dynamics_ubx = dynamics_ub;
dynamics_lbg = [zeros(num_interp, 1); torque_lb];
dynamics_ubg = [inf*ones(num_interp, 1); torque_ub];

smoothing_coeff = 0.8;

%% Simulate Distributed Multirobot System Control
for j=1:length(k)
    fprintf('\b\b\b\b\b\b\b%3.2f%%', (j/length(k))*100); % Simulation Progress
    p_j_array_database = q_next_k(1:dim_state-1, :);
    p_jdot_database = virtual_dq_k(1:dim_state-1, :);
    steady_torque = zeros(dim_state-1, 1);

    for i=1:mrs_size
        q_k(:, i) = q_next_k(:, i);
        measured_q = q_k(:, i);
        x_state_k = [measured_q; wheel_vel_i(:, i)];
        p_i = p_j_array_database(:, i);

        q_i_library(:, j, i) = measured_q;
        vel_library(:, j, i) = v_i(:, i);
        tau_library(:, j, i) = tau_i(:, i);
        wheeled_vel_library(:, j, i) = wheel_vel_i(:, i);

        % formation error system (ignore the dynamics of the leader)
        % if-statement to allow code to be collapsed
        %%{
        if (1)
            current_angle = q_next_virtual(3, i);
            P_diff_neighbours = ((p_i - p_j_array_database) - horzcat(Delta_ij{i, :}))*(adjacency(i, :)');
            leader_error = leader_neighbour(i)*((p_i - p_l(1:dim_state-1, j)) - Delta(:, i));
            global_formation_error = P_diff_neighbours + leader_error;

            %%% (with consensus)
            measured_eta_i = global_formation_error;
            h_i_dis = -((p_jdot_database*adjacency(i, :)') + leader_neighbour(i)*dp_l(1:dim_state-1, j));
            g_i_dis = sum(adjacency(i, :), 2) + leader_neighbour(i);
            
            %%% (no consensus): leader-follower error
            % measured_eta_i = (p_i - p_l(1:dim_state-1, j)) - Delta(:, i);
            % h_i_dis = -((p_jdot_database*silent_adjacency(i, :)')...
            %             + silent_leader_neighbour(i)*dp_l(1:dim_state-1, j));
            % g_i_dis = sum(silent_adjacency(i, :), 2) + silent_leader_neighbour(i);
            
            global_to_local = R_theta_func(measured_q); 
            local_formation_error = (global_to_local')*(global_formation_error - leader_error);
            formation_error(:, j, i) = abs(full(local_formation_error));

            auxiliary_input = g_i_dis\(-h_i_dis + formation_sys_opt_params(1:dim_state-1));
            h_CBF = cbf_constr_func(p_i, (p_i + auxiliary_input*dt), desired_gamma_CBF);

            formation_error_next = measured_eta_i;
            consensus_cartesian_velocity = (g_i_dis)\(-h_i_dis);
            consensus_velocity_magnitude = sqrt((consensus_cartesian_velocity')*consensus_cartesian_velocity);

            formation_error_cost = (formation_error_next')*Q_e*formation_error_next;
            formation_control_rate_cost = 0;
            formation_control_effort_cost = 0;
            formation_terminal_cost = 0;
            for r=1:M
                formation_error_next = dt_H_e*formation_error_next...
                                        + dt_G_e*formation_sys_opt_params(1:dim_state-1);
                running_velocity = g_i_dis\((-h_i_dis) + formation_sys_opt_params((r-1)*(dim_state-1)+1:r*(dim_state-1)));
                running_velocity_magnitude = sqrt((running_velocity')*running_velocity);

                if (r == 1)
                    control_rate = formation_sys_opt_params(1:(dim_state-1))...
                                    - formation_sys_opt_output(1:(dim_state-1), i);
                    formation_control_rate_cost = formation_control_rate_cost... 
                                                    + 0.5*(control_rate')*S_e*control_rate;
                    formation_error_cost = formation_error_cost...
                                              + (formation_error_next')...
                                                    *Q_e*formation_error_next;
                    velocity_error = (running_velocity_magnitude - consensus_velocity_magnitude);
                    formation_control_effort_cost = formation_control_effort_cost...
                                                    + penalise_vel*(velocity_error^2);
                elseif (r == M)
                    formation_terminal_cost = formation_beta_val*(formation_error_next')...
                                                *P_e*formation_error_next;
                else
                    control_rate = formation_sys_opt_params((r-1)*(dim_state-1)+1:r*(dim_state-1))...
                                    - formation_sys_opt_params((r-2)*(dim_state-1)+1:(r-1)*(dim_state-1));
                    formation_control_rate_cost = formation_control_rate_cost... 
                                                    + 0.5*(control_rate')*S_e*control_rate;
                    formation_error_cost = formation_error_cost...
                                              + (formation_error_next')...
                                                    *Q_e*formation_error_next;
                    formation_control_effort_cost = formation_control_effort_cost...
                                                    + penalise_vel*(velocity_error^2);
                end
            end
            
            % Define NLP problem
            formation_sys_objective = formation_error_cost...
                                        + formation_control_effort_cost...
                                        + formation_control_rate_cost...
                                        + formation_terminal_cost;
            formation_sys_constraint = [h_CBF; auxiliary_input];
            formation_sys_nlp = struct('x', formation_sys_opt_params, 'f', formation_sys_objective, 'g', formation_sys_constraint);
            
            % Create an NLP solver
            formation_sys_nlpsolve = nlpsol('solver', 'ipopt', formation_sys_nlp, opts);
            formation_sys_horizon_variables = formation_sys_nlpsolve('x0', formation_sys_opt_output(:, i),...
                                                 'lbx', form_err_lbx, 'ubx', form_err_ubx, ...
                                                 'lbg', form_err_lbg, 'ubg', form_err_ubg);
            formation_sys_opt_output(:, i) = full(formation_sys_horizon_variables.x);
            optimal_dp_i = g_i_dis\(-h_i_dis + formation_sys_opt_output(1:dim_state-1, i));

            % Desired Linear and Angular Velocity
            v_des_i = full(obtain_vel_func(current_angle)*optimal_dp_i);
            des_v_i = v_des_i(1);
            des_w_i = v_des_i(2);
            desired_wheel_vel = vel_forward\v_des_i;

            % Update positions of virtual followers
            virtual_state_f = q_solver('x0', q_next_virtual(:, i), 'p', v_des_i);
            q_next_virtual(:, i) = full(virtual_state_f.xf);

            % Desired trajectory to maintain formation
            desired_y = q_next_virtual(1:dim_state-1, i);
            desired_y_rate = optimal_dp_i;
            unfiltered_desired_y_accel = (desired_y_rate - desired_y_rate_history(:, i))/dt;
            desired_y_accel_history(:, i) = smoothing_coeff*unfiltered_desired_y_accel...
                                            + (1- smoothing_coeff)*desired_y_accel_history(:, i);
            desired_y_accel = desired_y_accel_history(:, i);
        end
        %}

        %%% Consensus evaluation
        v_i(:, i) = v_des_i;
        wheel_vel_i(:, i) = desired_wheel_vel;
        q_next_k(:, i) = q_next_virtual(:, i);
        virtual_dq_k(:, i) = [desired_y_rate; des_w_i];
    end
end

%% Plot Results
%%% trajectories
figure(1);
hold on;
plot(q_i_library(1, :, 1), q_i_library(2, :, 1), 'r', 'LineWidth', 1.5);
plot(q_i_library(1, :, 2), q_i_library(2, :, 2), 'b', 'LineWidth', 1.5);
plot(p_l(1, 1:length(k)), p_l(2, 1:length(k)), 'c--', 'LineWidth', 1.5);
rectangle('Position', [p_obs(1)-obstacle_radius,...
                       p_obs(2)-obstacle_radius,...
                       2*obstacle_radius, 2*obstacle_radius],...
                      'Curvature', [1, 1], 'FaceColor',[0 0 0]);
hold off;
legend('show', 'Location', 'best');
legend('Robot_1', 'Robot_2', 'Leader');
title('Formation trajectory', 'fontsize', 15);
xlabel('X coordinate [m]', 'fontSize', 12);
xlim([0 9]);
ylabel('Y coordinate [m]', 'fontSize', 12);
ylim([-0.5 1.5]);
pbaspect([diff(xlim) diff(ylim) 1]);
box on
grid on
grid minor
set(gca, 'fontsize', 12);
set(gca, 'TickDir', 'out');

%%% linear velocities
figure(2);
hold on;
plot(k, vel_library(1, :, 1), 'r', 'LineWidth', 1.5);
plot(k, vel_library(1, :, 2), 'b', 'LineWidth', 1.5);
plot(k, v_l(1, 1:length(k)), 'c--', 'LineWidth', 1.5);
hold off;
title('Linear Velocities', 'fontsize', 15);
xlabel('time [s]', 'fontSize', 12);
ylabel('Linear velocities v_i [m/s]', 'fontSize', 12);
legend('show', 'Location', 'northeast');
legend('Robot_1', 'Robot_2', 'Leader');
box on
grid on
grid minor
set(gca, 'fontsize', 12);
set(gca, 'TickDir', 'out');

%%% angular velocities
figure(3);
hold on;
plot(k, vel_library(2, :, 1), 'r', 'LineWidth', 1.5);
plot(k, vel_library(2, :, 2), 'b', 'LineWidth', 1.5);
plot(k, v_l(2, 1:length(k)), 'c--', 'LineWidth', 1.5);
hold off;
title('Angular Velocities', 'fontsize', 15);
xlabel('time [s]', 'fontSize', 12);
ylabel('Angular velocities $\omega_i$ [rad/s]', 'FontSize', 12, 'Interpreter', 'latex');
legend('show', 'Location', 'northeast');
legend('Robot_1', 'Robot_2', 'Leader');
box on
grid on
grid minor
set(gca, 'fontsize', 12);
set(gca, 'TickDir', 'out');

%%% X formation errors
figure(4);
hold on;
plot(k, formation_error(1, :, 1), 'r', 'LineWidth', 1.5);
plot(k, formation_error(1, :, 2), 'b', 'LineWidth', 1.5);
hold off;
title('Absolute longitudinal axis formation error (x)', 'fontsize', 15);
xlabel('time [s]', 'fontSize', 12);
xlim([0 9]);
ylabel('x-position error', 'fontSize', 12);
legend('show', 'Location', 'northeast');
legend('Robot_1', 'Robot_2');
box on
grid on
grid minor
set(gca, 'fontsize', 12);
set(gca, 'TickDir', 'out');

%%% Y formation errors
figure(5);
hold on;
plot(k, formation_error(2, :, 1), 'r', 'LineWidth', 1.5);
plot(k, formation_error(2, :, 2), 'b', 'LineWidth', 1.5);
hold off;
title('Absolute lateral axis formation error (y)', 'fontsize', 15);
xlabel('time [s]', 'fontSize', 12);
xlim([0 9]);
ylabel('y-position error', 'fontSize', 12);
legend('show', 'Location', 'northeast');
legend('Robot_1', 'Robot_2');
box on
grid on
grid minor
set(gca, 'fontsize', 12);
set(gca, 'TickDir', 'out');
