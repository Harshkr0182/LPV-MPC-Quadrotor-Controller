%% Enhanced script for Wavy Line Trajectory (Figures A.13-A.18)
% Improved accuracy version with tuned parameters
% Based on Mark Misin's Master's Thesis

clear all
close all
clc

%% ==================== PART 1: ENHANCED CONSTANTS ====================
fprintf('Initializing enhanced constants for better tracking...\n');

% Drone physical parameters (AscTec Hummingbird) - SAME as thesis
Ix = 0.0034;        % kg*m^2
Iy = 0.0034;        % kg*m^2
Iz = 0.006;         % kg*m^2
m = 0.698;          % kg
g = 9.81;           % m/s^2
Jtp = 1.302e-6;     % N*m*s^2

% ENHANCEMENT 1: Smaller sample time for better resolution
Ts = 0.05;           % s - Reduced from 0.1s for better tracking

% ENHANCEMENT 2: Tuned MPC weight matrices for better tracking
% Original was identity matrices - these are optimized
Q_position = 100;    % Not used directly but for reference
Q_angle = 50;        % Weight on angle tracking error
Q_angle_rate = 1;    % Weight on angular rate

% Form the weight matrices
Q = Q_angle * eye(3);                    % Output weights (angles)
R = 0.1 * eye(3);                         % Input change weights (smaller = more aggressive)
S = 2 * Q_angle * eye(3);                  % Terminal weight (higher = better convergence)

% Aerodynamic coefficients
ct = 7.6184e-8;     % N*s^2
cq = 2.6839e-9;     % N*m*s^2
l = 0.171;          % m

% ENHANCEMENT 3: Increased inner loop ratio for faster attitude control
innerDyn_length = 5;    % Inner loop iterations per outer loop (was 4)
hz = 5;                 % Prediction horizon (matches inner loop)

% Pole placement for position controller (more aggressive poles)
px = [-2+0j, -3+0j];    % Faster poles for x-direction
py = [-2+0j, -3+0j];    % Faster poles for y-direction
pz = [-2+0j, -3+0j];    % Faster poles for z-direction

% Input bounds for quadprog (slightly relaxed for better maneuverability)
lb = [-1.0; -1.0; -1.0];   % Lower bounds for delta U
ub = [1.0; 1.0; 1.0];      % Upper bounds for delta U

% Maximum input saturation (for anti-windup)
U_max = [10; 0.5; 0.5; 0.5];  % Max absolute inputs [U1, U2, U3, U4]
U_min = [0; -0.5; -0.5; -0.5]; % Min absolute inputs

fprintf('Enhanced constants initialized.\n\n');

%% ==================== PART 2: WAVY LINE TRAJECTORY GENERATION ====================
fprintf('Generating wavy line trajectory (Figures A.13-A.18)...\n');

% Time vector - 60 seconds for wavy line (as per thesis)
t_end = 60;
t = 0:(Ts*innerDyn_length):t_end;
t_fine = 0:0.01:t_end;  % Fine time vector for smooth reference

% ENHANCEMENT 4: Smooth trajectory generation with proper derivatives
% Wavy line parameters (based on thesis)
x_amp = 2;          % Amplitude in x
y_amp = 2;          % Amplitude in y
z_amp = 3;          % Height variation
freq = 0.1;         % Frequency of waves

% Generate smooth reference using analytical functions (for exact derivatives)
t_smooth = t;
x_ref_func = @(t) 2*t/20 + 1;  % Linear in x with offset
y_ref_func = @(t) 2*sin(0.15*t);  % Sinusoidal in y
z_ref_func = @(t) 2 + 1.5*sin(0.1*t);  % Sinusoidal in z

% Position references
x_ref_coords = x_ref_func(t_smooth);
y_ref_coords = y_ref_func(t_smooth);
z_ref_coords = z_ref_func(t_smooth);

% Analytical derivatives for exact velocity references
x_dot_func = @(t) 2/20 * ones(size(t));  % Constant velocity
y_dot_func = @(t) 2*0.15*cos(0.15*t);    % Derivative of sin
z_dot_func = @(t) 1.5*0.1*cos(0.1*t);    % Derivative of sin

x_dot_ref = x_dot_func(t_smooth);
y_dot_ref = y_dot_func(t_smooth);
z_dot_ref = z_dot_func(t_smooth);

% Calculate reference yaw angle (psi) - pointing in direction of motion
psi_ref = zeros(1, length(x_ref_coords));
for i = 1:length(psi_ref)
    psi_ref(i) = atan2(y_dot_ref(i), x_dot_ref(i));
end

% Ensure continuous yaw angle (unwrap)
psi_ref = unwrap(psi_ref);

% Format reference signals
X_ref = [t' x_ref_coords'];
X_dot_ref = [t' x_dot_ref'];
Y_ref = [t' y_ref_coords'];
Y_dot_ref = [t' y_dot_ref'];
Z_ref = [t' z_ref_coords'];
Z_dot_ref = [t' z_dot_ref'];
psi_ref_matrix = [t' psi_ref'];

% Time vector for inner loop (angles)
t_angles = 0:Ts:t_end;

plotl = length(t);  % Number of outer control loop iterations

fprintf('Wavy line trajectory generated. %d outer loop iterations.\n\n', plotl);

%% ==================== PART 3: INITIAL CONDITIONS ====================
fprintf('Setting initial conditions...\n');

% Initial velocities in body frame
u = 0; v = 0; w = 0;
p = 0; q = 0; r = 0;

% Initial positions and angles (start on trajectory)
x_init = x_ref_coords(1);
y_init = y_ref_coords(1);
z_init = z_ref_coords(1);
phi_init = 0;
theta_init = 0;
psi_init = psi_ref(1);

% Initial state vector
states = [u, v, w, p, q, r, x_init, y_init, z_init, phi_init, theta_init, psi_init];
states_total = states;

% Initial rotor speeds (rad/s)
omega1 = 3200;  % Slightly higher for better initial thrust
omega2 = 3200;
omega3 = 3200;
omega4 = 3200;

% Initial control inputs
U1 = ct * (omega1^2 + omega2^2 + omega3^2 + omega4^2);
U2 = ct * l * (omega4^2 - omega2^2);
U3 = ct * l * (omega3^2 - omega1^2);
U4 = cq * (-omega1^2 + omega2^2 - omega3^2 + omega4^2);

UTotal = [U1, U2, U3, U4];

% Total rotational velocity
omega_total = -omega1 + omega2 - omega3 + omega4;

% Storage arrays
ref_angles_total = [phi_init, theta_init, psi_init];
velocityXYZ_total = [x_dot_ref(1), y_dot_ref(1), z_dot_ref(1)];
input_history = UTotal;
error_history = [];

fprintf('Initial conditions set.\n\n');

%% ==================== PART 4: MAIN CONTROL LOOP ====================
fprintf('Starting enhanced main control loop...\n');
fprintf('Progress: 0%%');

% ENHANCEMENT 5: Add integral action for position controller
ex_integral = 0;
ey_integral = 0;
ez_integral = 0;

for i_global = 1:plotl-1
    
    % Update progress
    if mod(i_global, round(plotl/10)) == 0
        fprintf('\b\b\b\b%d%%', round(100*i_global/plotl));
    end
    
    %% 4.1: ENHANCED POSITION CONTROLLER with integral action
    X_ref_current = X_ref(i_global+1, 2);
    X_dot_ref_current = X_dot_ref(i_global+1, 2);
    Y_ref_current = Y_ref(i_global+1, 2);
    Y_dot_ref_current = Y_dot_ref(i_global+1, 2);
    Z_ref_current = Z_ref(i_global+1, 2);
    Z_dot_ref_current = Z_dot_ref(i_global+1, 2);
    Psi_ref_current = psi_ref_matrix(i_global+1, 2);
    
    % Call enhanced position controller
    [phi_ref, theta_ref, U1, ex_integral, ey_integral, ez_integral] = ...
        pos_controller_enhanced(...
        X_ref_current, X_dot_ref_current, ...
        Y_ref_current, Y_dot_ref_current, ...
        Z_ref_current, Z_dot_ref_current, ...
        Psi_ref_current, states, m, g, px, py, pz, ...
        ex_integral, ey_integral, ez_integral, Ts*innerDyn_length);
    
    % Apply input saturation (anti-windup)
    U1 = max(min(U1, U_max(1)), U_min(1));
    
    % Create reference angles for inner loop
    Phi_ref_vec = phi_ref * ones(innerDyn_length+1, 1);
    Theta_ref_vec = theta_ref * ones(innerDyn_length+1, 1);
    Psi_ref_vec = Psi_ref_current * ones(innerDyn_length+1, 1);
    
    % Store reference angles
    ref_angles_total = [ref_angles_total; Phi_ref_vec(2:end), Theta_ref_vec(2:end), Psi_ref_vec(2:end)];
    
    %% 4.2: Create reference vector for MPC
    refSignals = zeros(length(Phi_ref_vec(:,1)) * 3, 1);
    k_ref_local = 1;
    for i = 1:3:length(refSignals)
        refSignals(i) = Phi_ref_vec(k_ref_local, 1);
        refSignals(i+1) = Theta_ref_vec(k_ref_local, 1);
        refSignals(i+2) = Psi_ref_vec(k_ref_local, 1);
        k_ref_local = k_ref_local + 1;
    end
    
    %% 4.3: INNER LOOP - ENHANCED LPV-MPC ATTITUDE CONTROLLER
    k_ref_local = 1;
    
    for i_inner = 1:innerDyn_length
        % Get discrete LPV model
        [Ad, Bd, Cd, Dd, x_dot, y_dot, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot] = ...
            LPV_cont_discrete_enhanced(states, Ix, Iy, Iz, Jtp, Ts, omega_total);
        
        % Store velocities
        velocityXYZ_total = [velocityXYZ_total; [x_dot, y_dot, z_dot]];
        
        % Current augmented state
        x_aug_t = [phi; phi_dot; theta; theta_dot; psi; psi_dot; U2; U3; U4];
        
        k_ref_local = k_ref_local + 3;
        
        % Get reference signals for horizon
        if k_ref_local + 3 * hz - 1 <= length(refSignals)
            r_ref = refSignals(k_ref_local : k_ref_local + 3 * hz - 1);
            hz_current = hz;
        else
            r_ref = refSignals(k_ref_local : length(refSignals));
            hz_current = length(r_ref) / 3;
        end
        
        % Generate MPC matrices
        [Hdb, Fdbt] = MPC_simplification_enhanced(Ad, Bd, Cd, Dd, hz_current, Q, S, R);
        
        % Prepare for quadprog
        ft = [x_aug_t', r_ref'] * Fdbt;
        
        % Call quadprog
        options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
        [du, ~] = quadprog(Hdb, ft', [], [], [], [], lb, ub, [], options);
        
        % Update control inputs
        if ~isempty(du)
            U2 = U2 + du(1);
            U3 = U3 + du(2);
            U4 = U4 + du(3);
        end
        
        % Apply input saturation
        U2 = max(min(U2, U_max(2)), U_min(2));
        U3 = max(min(U3, U_max(3)), U_min(3));
        U4 = max(min(U4, U_max(4)), U_min(4));
        
        % Compute new rotor speeds
        U1C = U1 / ct;
        U2C = U2 / (ct * l);
        U3C = U3 / (ct * l);
        U4C = U4 / cq;
        
        omega4P2 = (U1C + 2*U2C + U4C) / 4;
        omega3P2 = (U4C + 2*omega4P2 - U2C + U3C) / 2;
        omega2P2 = omega4P2 - U2C;
        omega1P2 = omega3P2 - U3C;
        
        % Use abs and sqrt with protection
        omega1 = sqrt(max(abs(omega1P2), 1e-6));
        omega2 = sqrt(max(abs(omega2P2), 1e-6));
        omega3 = sqrt(max(abs(omega3P2), 1e-6));
        omega4 = sqrt(max(abs(omega4P2), 1e-6));
        
        % Update total omega
        omega_total = -omega1 + omega2 - omega3 + omega4;
        
        % Store inputs
        input_history = [input_history; U1, U2, U3, U4];
        
        %% 4.4: SIMULATE NONLINEAR DRONE MODEL with higher accuracy
        T_start = Ts * (i_global-1) + (i_inner-1) * (Ts/innerDyn_length);
        T_end = Ts * (i_global-1) + i_inner * (Ts/innerDyn_length);
        
        % Use variable step with higher accuracy settings
        options_ode = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
        [~, x_new] = ode45(@(t,x) nonlinear_drone_model_enhanced(t, x, [U1, U2, U3, U4], ...
            Ix, Iy, Iz, m, g, Jtp, omega_total), [T_start, T_end], states, options_ode);
        
        % Update states
        states = x_new(end, :);
        states_total = [states_total; states];
        
        % Calculate and store tracking error
        current_error = norm(states(7:9) - [X_ref_current, Y_ref_current, Z_ref_current]);
        error_history = [error_history; current_error];
    end
end

fprintf('\nMain control loop completed.\n');
fprintf('Average tracking error: %.4f m\n', mean(error_history));
fprintf('Max tracking error: %.4f m\n\n', max(error_history));

%% ==================== PART 5: ENHANCED PLOTTING ====================
fprintf('Generating enhanced plots (Figures A.13-A.18)...\n');

% FIGURE 1: Flight trajectory - wavy line (like Figure A.13)
figure('Position', [50, 50, 900, 700]);
plot3(X_ref(:,2), Y_ref(:,2), Z_ref(:,2), '--b', 'LineWidth', 2.5);
hold on;
plot3(states_total(1:innerDyn_length:end,7), ...
      states_total(1:innerDyn_length:end,8), ...
      states_total(1:innerDyn_length:end,9), 'r', 'LineWidth', 2);
grid on;
xlabel('x- position [m]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('y- position [m]', 'FontSize', 14, 'FontWeight', 'bold');
zlabel('z- position [m]', 'FontSize', 14, 'FontWeight', 'bold');
title('Flight trajectory - wavy line in 3D', 'FontSize', 16, 'FontWeight', 'bold');
legend({'Reference', 'Actual trajectory'}, 'Location', 'northeast', 'FontSize', 12);
view(45, 30);
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

% FIGURE 2: x and x_dot (like Figure A.14)
figure('Position', [100, 100, 900, 700]);
subplot(2,1,1);
plot(t(1:plotl), X_ref(1:plotl,2), '--b', 'LineWidth', 2.5);
hold on;
plot(t(1:plotl), states_total(1:innerDyn_length:end,7), 'r', 'LineWidth', 2);
grid on;
xlabel('time [s]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('x- position [m]', 'FontSize', 14, 'FontWeight', 'bold');
legend({'x- ref', 'x- position'}, 'Location', 'northeast', 'FontSize', 12);
title('x position tracking', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

subplot(2,1,2);
plot(t(1:plotl), X_dot_ref(1:plotl,2), '--b', 'LineWidth', 2.5);
hold on;
plot(t(1:plotl), velocityXYZ_total(1:innerDyn_length:end,1), 'r', 'LineWidth', 2);
grid on;
xlabel('time [s]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('x- velocity [m/s]', 'FontSize', 14, 'FontWeight', 'bold');
legend({'x- dot- ref', 'x- velocity'}, 'Location', 'northeast', 'FontSize', 12);
title('x velocity tracking', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

% FIGURE 3: y and y_dot (like Figure A.15)
figure('Position', [150, 150, 900, 700]);
subplot(2,1,1);
plot(t(1:plotl), Y_ref(1:plotl,2), '--b', 'LineWidth', 2.5);
hold on;
plot(t(1:plotl), states_total(1:innerDyn_length:end,8), 'r', 'LineWidth', 2);
grid on;
xlabel('time [s]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('y- position [m]', 'FontSize', 14, 'FontWeight', 'bold');
legend({'y- ref', 'y- position'}, 'Location', 'northeast', 'FontSize', 12);
title('y position tracking', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

subplot(2,1,2);
plot(t(1:plotl), Y_dot_ref(1:plotl,2), '--b', 'LineWidth', 2.5);
hold on;
plot(t(1:plotl), velocityXYZ_total(1:innerDyn_length:end,2), 'r', 'LineWidth', 2);
grid on;
xlabel('time [s]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('y- velocity [m/s]', 'FontSize', 14, 'FontWeight', 'bold');
legend({'y- dot- ref', 'y- velocity'}, 'Location', 'northeast', 'FontSize', 12);
title('y velocity tracking', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

% FIGURE 4: z and z_dot (like Figure A.16)
figure('Position', [200, 200, 900, 700]);
subplot(2,1,1);
plot(t(1:plotl), Z_ref(1:plotl,2), '--b', 'LineWidth', 2.5);
hold on;
plot(t(1:plotl), states_total(1:innerDyn_length:end,9), 'r', 'LineWidth', 2);
grid on;
xlabel('time [s]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('z- position [m]', 'FontSize', 14, 'FontWeight', 'bold');
legend({'z- ref', 'z- position'}, 'Location', 'northeast', 'FontSize', 12);
title('z position tracking', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

subplot(2,1,2);
plot(t(1:plotl), Z_dot_ref(1:plotl,2), '--b', 'LineWidth', 2.5);
hold on;
plot(t(1:plotl), velocityXYZ_total(1:innerDyn_length:end,3), 'r', 'LineWidth', 2);
grid on;
xlabel('time [s]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('z- velocity [m/s]', 'FontSize', 14, 'FontWeight', 'bold');
legend({'z- dot- ref', 'z- velocity'}, 'Location', 'northeast', 'FontSize', 12);
title('z velocity tracking', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

% FIGURE 5: Angles phi, theta, psi (like Figure A.17)
figure('Position', [250, 250, 900, 900]);
subplot(3,1,1);
plot(t_angles(1:length(ref_angles_total(:,1))), ref_angles_total(:,1), '--b', 'LineWidth', 2.5);
hold on;
plot(t_angles(1:length(states_total(:,10))), states_total(:,10), 'r', 'LineWidth', 2);
grid on;
xlabel('time [s]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('phi [rad]', 'FontSize', 14, 'FontWeight', 'bold');
legend({'phi- ref', 'phi- angle'}, 'Location', 'northeast', 'FontSize', 12);
title('Roll angle tracking', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

subplot(3,1,2);
plot(t_angles(1:length(ref_angles_total(:,2))), ref_angles_total(:,2), '--b', 'LineWidth', 2.5);
hold on;
plot(t_angles(1:length(states_total(:,11))), states_total(:,11), 'r', 'LineWidth', 2);
grid on;
xlabel('time [s]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('theta [rad]', 'FontSize', 14, 'FontWeight', 'bold');
legend({'theta- ref', 'theta- angle'}, 'Location', 'northeast', 'FontSize', 12);
title('Pitch angle tracking', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

subplot(3,1,3);
plot(t_angles(1:length(ref_angles_total(:,3))), ref_angles_total(:,3), '--b', 'LineWidth', 2.5);
hold on;
plot(t_angles(1:length(states_total(:,12))), states_total(:,12), 'r', 'LineWidth', 2);
grid on;
xlabel('time [s]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('psi [rad]', 'FontSize', 14, 'FontWeight', 'bold');
legend({'psi- ref', 'psi- angle'}, 'Location', 'northeast', 'FontSize', 12);
title('Yaw angle tracking', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

% FIGURE 6: Control inputs U1, U2, U3, U4 (like Figure A.18)
figure('Position', [300, 300, 900, 900]);
subplot(4,1,1);
plot(t_angles(1:length(input_history(:,1))), input_history(:,1), 'k', 'LineWidth', 1.5);
grid on;
xlabel('time [s]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('U1 [N]', 'FontSize', 14, 'FontWeight', 'bold');
title('Thrust input', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

subplot(4,1,2);
plot(t_angles(1:length(input_history(:,2))), input_history(:,2), 'k', 'LineWidth', 1.5);
grid on;
xlabel('time [s]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('U2 [Nm]', 'FontSize', 14, 'FontWeight', 'bold');
title('Roll moment', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

subplot(4,1,3);
plot(t_angles(1:length(input_history(:,3))), input_history(:,3), 'k', 'LineWidth', 1.5);
grid on;
xlabel('time [s]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('U3 [Nm]', 'FontSize', 14, 'FontWeight', 'bold');
title('Pitch moment', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

subplot(4,1,4);
plot(t_angles(1:length(input_history(:,4))), input_history(:,4), 'k', 'LineWidth', 1.5);
grid on;
xlabel('time [s]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('U4 [Nm]', 'FontSize', 14, 'FontWeight', 'bold');
title('Yaw moment', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

% Additional Figure: Tracking Error Analysis
figure('Position', [350, 350, 900, 400]);
subplot(1,2,1);
plot(t_angles(1:length(error_history)), error_history, 'b', 'LineWidth', 1.5);
grid on;
xlabel('time [s]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Position Error [m]', 'FontSize', 14, 'FontWeight', 'bold');
title('Tracking Error vs Time', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

subplot(1,2,2);
histogram(error_history, 50, 'FaceColor', 'b', 'EdgeColor', 'k');
xlabel('Position Error [m]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Frequency', 'FontSize', 14, 'FontWeight', 'bold');
title('Error Distribution', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

fprintf('Plotting completed.\n');
fprintf('Simulation finished successfully!\n');
fprintf('Average tracking error: %.4f m\n', mean(error_history));