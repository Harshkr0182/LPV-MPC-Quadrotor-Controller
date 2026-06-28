%% Main script to reproduce Figure A.1 - Extended Spiral Trajectory
% Based on Mark Misin's Master's Thesis
% This script implements the combined Position Controller + LPV-MPC Attitude Controller

clear all
close all
clc

%% ==================== PART 1: INITIAL CONSTANTS ====================
fprintf('Initializing constants...\n');

% Drone physical parameters (AscTec Hummingbird)
Ix = 0.0034;        % kg*m^2 - Moment of inertia around x-axis
Iy = 0.0034;        % kg*m^2 - Moment of inertia around y-axis
Iz = 0.006;         % kg*m^2 - Moment of inertia around z-axis
m = 0.698;          % kg - Drone mass
g = 9.81;           % m/s^2 - Gravity
Jtp = 1.302e-6;     % N*m*s^2 - Total rotational moment of inertia around propeller axis
Ts = 0.1;           % s - Sample time

% MPC weight matrices (identity matrices as per thesis)
Q = eye(3);         % weights for outputs
S = eye(3);         % weights for final horizon outputs
R = eye(3);         % weights for inputs

% Aerodynamic coefficients
ct = 7.6184e-8;     % N*s^2 - Thrust coefficient
cq = 2.6839e-9;     % N*m*s^2 - Drag coefficient
l = 0.171;          % m - Distance from center to propeller

% Controller parameters
controlled_states = 3;  % Number of controlled states (phi, theta, psi)
hz = 4;                 % Prediction horizon
innerDyn_length = 4;    % Inner loop iterations per outer loop (4x faster)

% Pole placement for position controller (negative real poles)
px = [-1+0j, -2+0j];    % Poles for x-direction
py = [-1+0j, -2+0j];    % Poles for y-direction
pz = [-1+0j, -2+0j];    % Poles for z-direction

% Input bounds for quadprog
lb = [-0.5; -0.5; -0.5];   % Lower bounds for delta U
ub = [0.5; 0.5; 0.5];      % Upper bounds for delta U

fprintf('Constants initialized.\n\n');

%% ==================== PART 2: TRAJECTORY GENERATION ====================
fprintf('Generating extended spiral trajectory...\n');

% Time vector (100 seconds as in thesis)
t_end = 100;
t = 0:(Ts*innerDyn_length):t_end;  % Outer loop time vector

% Extended spiral parameters (as in Appendix A.1)
r = 2;                  % Base radius (will expand with time)
f = 0.025;              % Frequency for angular velocity
height_i = 2;           % Initial height (m)
height_f = 5;           % Final height (m)

% Generate extended spiral: radius increases with time
alpha = 2*pi*f.*t;
d_height = height_f - height_i;

% Extended spiral: x = (r + time_factor)*cos(alpha), y = (r + time_factor)*sin(alpha)
% This creates an expanding spiral as in Figure A.1
time_factor = 0.1 * t;  % Radius expands with time

x_ref_coords = (r + time_factor) .* cos(alpha);
y_ref_coords = (r + time_factor) .* sin(alpha);
z_ref_coords = height_i + d_height/t_end * t;

% Calculate reference velocities (finite difference)
dx = [x_ref_coords(2)-x_ref_coords(1), x_ref_coords(2:end)-x_ref_coords(1:end-1)];
dy = [y_ref_coords(2)-y_ref_coords(1), y_ref_coords(2:end)-y_ref_coords(1:end-1)];
dz = [z_ref_coords(2)-z_ref_coords(1), z_ref_coords(2:end)-z_ref_coords(1:end-1)];

x_dot_ref = dx * (1/(Ts*innerDyn_length));
y_dot_ref = dy * (1/(Ts*innerDyn_length));
z_dot_ref = round(dz * (1/(Ts*innerDyn_length)), 8);

% Calculate reference yaw angle (psi)
psi_ref = zeros(1, length(x_ref_coords));
psi_ref(1) = atan2(y_ref_coords(1), x_ref_coords(1)) + pi/2;
psi_ref(2:end) = atan2(dy(2:end), dx(2:end));

% Ensure continuous yaw angle (unwrap)
for i = 1:length(psi_ref)
    if psi_ref(i) < 0
        psi_ref(i) = 2*pi - abs(psi_ref(i));
    end
end

for i = 2:length(psi_ref)
    if abs(psi_ref(i) - psi_ref(i-1)) > pi
        psi_ref(i:end) = psi_ref(i:end) + 2*pi;
    end
end

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

fprintf('Trajectory generated. %d outer loop iterations.\n\n', plotl);

%% ==================== PART 3: INITIAL CONDITIONS ====================
fprintf('Setting initial conditions...\n');

% Initial velocities in body frame
u = 0; v = 0; w = 0;    % Linear velocities (m/s)
p = 0; q = 0; r = 0;    % Angular velocities (rad/s)

% Initial positions and angles (as in thesis)
x_init = 0;             % Initial x position (m)
y_init = -1;            % Initial y position (m) - offset from trajectory
z_init = 0;             % Initial z position (m)
phi_init = 0;           % Initial roll angle (rad)
theta_init = 0;         % Initial pitch angle (rad)
psi_init = psi_ref(1,2); % Initial yaw angle from planner

% Initial state vector [u, v, w, p, q, r, x, y, z, phi, theta, psi]
states = [u, v, w, p, q, r, x_init, y_init, z_init, phi_init, theta_init, psi_init];
states_total = states;

% Initial rotor speeds (rad/s) - from thesis: 3000 rad/s at t = -1
omega1 = 3000;
omega2 = 3000;
omega3 = 3000;
omega4 = 3000;

% Initial control inputs based on rotor speeds
U1 = ct * (omega1^2 + omega2^2 + omega3^2 + omega4^2);
U2 = ct * l * (omega4^2 - omega2^2);
U3 = ct * l * (omega3^2 - omega1^2);
U4 = cq * (-omega1^2 + omega2^2 - omega3^2 + omega4^2);

UTotal = [U1, U2, U3, U4];

% Total rotational velocity of rotors
omega_total = -omega1 + omega2 - omega3 + omega4;

% Storage arrays
ref_angles_total = [phi_init, theta_init, psi_init];
velocityXYZ_total = [x_dot_ref(1), y_dot_ref(1), z_dot_ref(1)];

fprintf('Initial conditions set.\n\n');

%% ==================== PART 4: MAIN CONTROL LOOP ====================
fprintf('Starting main control loop...\n');
fprintf('Progress: 0%%');

for i_global = 1:plotl-1
    
    % Update progress
    if mod(i_global, round(plotl/10)) == 0
        fprintf('\b\b\b\b%d%%', round(100*i_global/plotl));
    end
    
    %% 4.1: POSITION CONTROLLER (Feedback Linearization)
    % Get reference values for current time step
    X_ref_current = X_ref(i_global+1, 2);
    X_dot_ref_current = X_dot_ref(i_global+1, 2);
    Y_ref_current = Y_ref(i_global+1, 2);
    Y_dot_ref_current = Y_dot_ref(i_global+1, 2);
    Z_ref_current = Z_ref(i_global+1, 2);
    Z_dot_ref_current = Z_dot_ref(i_global+1, 2);
    Psi_ref_current = psi_ref_matrix(i_global+1, 2);
    
    % Call position controller
    [phi_ref, theta_ref, U1] = pos_controller(...
        X_ref_current, X_dot_ref_current, ...
        Y_ref_current, Y_dot_ref_current, ...
        Z_ref_current, Z_dot_ref_current, ...
        Psi_ref_current, states, m, g, px, py, pz);
    
    % Create reference angles for inner loop (constant during inner iterations)
    Phi_ref_vec = phi_ref * ones(innerDyn_length+1, 1);
    Theta_ref_vec = theta_ref * ones(innerDyn_length+1, 1);
    Psi_ref_vec = Psi_ref_current * ones(innerDyn_length+1, 1);
    
    % Store reference angles
    ref_angles_total = [ref_angles_total; Phi_ref_vec(2:end), Theta_ref_vec(2:end), Psi_ref_vec(2:end)];
    
    %% 4.2: Create reference vector for MPC
    refSignals = zeros(length(Phi_ref_vec(:,1)) * controlled_states, 1);
    k_ref_local = 1;
    for i = 1:controlled_states:length(refSignals)
        refSignals(i) = Phi_ref_vec(k_ref_local, 1);
        refSignals(i+1) = Theta_ref_vec(k_ref_local, 1);
        refSignals(i+2) = Psi_ref_vec(k_ref_local, 1);
        k_ref_local = k_ref_local + 1;
    end
    
    %% 4.3: INNER LOOP - LPV-MPC ATTITUDE CONTROLLER
    k_ref_local = 1;  % Reset for reading reference signals
    
    for i_inner = 1:innerDyn_length
        % Get discrete LPV model for attitude
        [Ad, Bd, Cd, Dd, x_dot, y_dot, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot] = ...
            LPV_cont_discrete(states, Ix, Iy, Iz, Jtp, Ts, omega_total);
        
        % Store velocities
        velocityXYZ_total = [velocityXYZ_total; [x_dot, y_dot, z_dot]];
        
        % Current augmented state for MPC [phi; phi_dot; theta; theta_dot; psi; psi_dot; U2; U3; U4]
        x_aug_t = [phi; phi_dot; theta; theta_dot; psi; psi_dot; U2; U3; U4];
        
        k_ref_local = k_ref_local + controlled_states;
        
        % Get reference signals for the horizon
        if k_ref_local + controlled_states * hz - 1 <= length(refSignals)
            r_ref = refSignals(k_ref_local : k_ref_local + controlled_states * hz - 1);
            hz_current = hz;
        else
            r_ref = refSignals(k_ref_local : length(refSignals));
            hz_current = length(r_ref) / controlled_states;
        end
        
        % Generate MPC simplification matrices
        [Hdb, Fdbt] = MPC_simplification(Ad, Bd, Cd, Dd, hz_current, Q, S, R);
        
        % Prepare for quadprog
        ft = [x_aug_t', r_ref'] * Fdbt;
        
        % Check if Hdb is positive definite
        [~, p] = chol(Hdb);
        if p ~= 0
            warning('Hdb is NOT positive definite at iteration %d, inner %d', i_global, i_inner);
        end
        
        % Call quadprog solver
        options = optimoptions('quadprog', 'Display', 'off');
        [du, ~] = quadprog(Hdb, ft', [], [], [], [], lb, ub, [], options);
        
        % Update control inputs
        U2 = U2 + du(1);
        U3 = U3 + du(2);
        U4 = U4 + du(3);
        
        % Compute new rotor speeds based on updated U-s
        U1C = U1 / ct;
        U2C = U2 / (ct * l);
        U3C = U3 / (ct * l);
        U4C = U4 / cq;
        
        omega4P2 = (U1C + 2*U2C + U4C) / 4;
        omega3P2 = (U4C + 2*omega4P2 - U2C + U3C) / 2;
        omega2P2 = omega4P2 - U2C;
        omega1P2 = omega3P2 - U3C;
        
        omega1 = sqrt(abs(omega1P2));  % Use abs to avoid imaginary numbers
        omega2 = sqrt(abs(omega2P2));
        omega3 = sqrt(abs(omega3P2));
        omega4 = sqrt(abs(omega4P2));
        
        % Update total omega
        omega_total = -omega1 + omega2 - omega3 + omega4;
        
        % Store inputs
        UTotal = [UTotal; U1, U2, U3, U4];
        
        %% 4.4: SIMULATE NONLINEAR DRONE MODEL
        % Time span for integration (30 steps within one sample time)
        T_start = Ts * (i_global-1) + (i_inner-1) * (Ts/innerDyn_length);
        T_end = Ts * (i_global-1) + i_inner * (Ts/innerDyn_length);
        T_span = T_start:(Ts/30):T_end;
        
        % Integrate using ode45
        [~, x_new] = ode45(@(t,x) nonlinear_drone_model(t, x, [U1, U2, U3, U4], ...
            Ix, Iy, Iz, m, g, Jtp, omega_total), T_span, states);
        
        % Update states with final values
        states = x_new(end, :);
        states_total = [states_total; states];
        
        % Check for imaginary parts
        if any(imag(states) ~= 0)
            warning('Imaginary part detected - resetting to real values');
            states = real(states);
        end
    end
end

fprintf('\nMain control loop completed.\n\n');

%% ==================== PART 5: PLOTTING ====================
fprintf('Generating plots...\n');

% FIGURE 1: Flight trajectory - extended spiral (exactly like Figure A.1)
figure('Position', [100, 100, 800, 600]);
plot3(X_ref(:,2), Y_ref(:,2), Z_ref(:,2), '--b', 'LineWidth', 2);
hold on;
plot3(states_total(1:innerDyn_length:end,7), ...
      states_total(1:innerDyn_length:end,8), ...
      states_total(1:innerDyn_length:end,9), 'r', 'LineWidth', 1.5);
grid on;
xlabel('x- position [m]', 'FontSize', 12);
ylabel('y- position [m]', 'FontSize', 12);
zlabel('z- position [m]', 'FontSize', 12);
title('Flight trajectory - extended spiral', 'FontSize', 14);
legend({'position-ref', 'position'}, 'Location', 'northeast', 'FontSize', 11);
view(45, 30);  % Set a nice 3D view angle

% FIGURE 2: x and x_dot (like Figure A.2)
figure('Position', [100, 100, 800, 600]);
subplot(2,1,1);
plot(t(1:plotl), X_ref(1:plotl,2), '--b', 'LineWidth', 2);
hold on;
plot(t(1:plotl), states_total(1:innerDyn_length:end,7), 'r', 'LineWidth', 1.5);
grid on;
xlabel('time [s]', 'FontSize', 12);
ylabel('x- position [m]', 'FontSize', 12);
legend({'x- ref', 'x- position'}, 'Location', 'northeast', 'FontSize', 11);
title('x position tracking', 'FontSize', 12);

subplot(2,1,2);
plot(t(1:plotl), X_dot_ref(1:plotl,2), '--b', 'LineWidth', 2);
hold on;
plot(t(1:plotl), velocityXYZ_total(1:innerDyn_length:end,1), 'r', 'LineWidth', 1.5);
grid on;
xlabel('time [s]', 'FontSize', 12);
ylabel('x- velocity [m/s]', 'FontSize', 12);
legend({'x- dot- ref', 'x- velocity'}, 'Location', 'northeast', 'FontSize', 11);
title('x velocity tracking', 'FontSize', 12);

% FIGURE 3: y and y_dot (like Figure A.3)
figure('Position', [100, 100, 800, 600]);
subplot(2,1,1);
plot(t(1:plotl), Y_ref(1:plotl,2), '--b', 'LineWidth', 2);
hold on;
plot(t(1:plotl), states_total(1:innerDyn_length:end,8), 'r', 'LineWidth', 1.5);
grid on;
xlabel('time [s]', 'FontSize', 12);
ylabel('y- position [m]', 'FontSize', 12);
legend({'y- ref', 'y- position'}, 'Location', 'northeast', 'FontSize', 11);
title('y position tracking', 'FontSize', 12);

subplot(2,1,2);
plot(t(1:plotl), Y_dot_ref(1:plotl,2), '--b', 'LineWidth', 2);
hold on;
plot(t(1:plotl), velocityXYZ_total(1:innerDyn_length:end,2), 'r', 'LineWidth', 1.5);
grid on;
xlabel('time [s]', 'FontSize', 12);
ylabel('y- velocity [m/s]', 'FontSize', 12);
legend({'y- dot- ref', 'y- velocity'}, 'Location', 'northeast', 'FontSize', 11);
title('y velocity tracking', 'FontSize', 12);

% FIGURE 4: z and z_dot (like Figure A.4)
figure('Position', [100, 100, 800, 600]);
subplot(2,1,1);
plot(t(1:plotl), Z_ref(1:plotl,2), '--b', 'LineWidth', 2);
hold on;
plot(t(1:plotl), states_total(1:innerDyn_length:end,9), 'r', 'LineWidth', 1.5);
grid on;
xlabel('time [s]', 'FontSize', 12);
ylabel('z- position [m]', 'FontSize', 12);
legend({'z- ref', 'z- position'}, 'Location', 'northeast', 'FontSize', 11);
title('z position tracking', 'FontSize', 12);

subplot(2,1,2);
plot(t(1:plotl), Z_dot_ref(1:plotl,2), '--b', 'LineWidth', 2);
hold on;
plot(t(1:plotl), velocityXYZ_total(1:innerDyn_length:end,3), 'r', 'LineWidth', 1.5);
grid on;
xlabel('time [s]', 'FontSize', 12);
ylabel('z- velocity [m/s]', 'FontSize', 12);
legend({'z- dot- ref', 'z- velocity'}, 'Location', 'northeast', 'FontSize', 11);
title('z velocity tracking', 'FontSize', 12);

% FIGURE 5: Angles phi, theta, psi (like Figure A.5)
figure('Position', [100, 100, 800, 600]);
subplot(3,1,1);
plot(t_angles(1:length(ref_angles_total(:,1))), ref_angles_total(:,1), '--b', 'LineWidth', 2);
hold on;
plot(t_angles(1:length(states_total(:,10))), states_total(:,10), 'r', 'LineWidth', 1.5);
grid on;
xlabel('time [s]', 'FontSize', 12);
ylabel('phi [rad]', 'FontSize', 12);
legend({'phi- ref', 'phi- angle'}, 'Location', 'northeast', 'FontSize', 11);
title('Roll angle tracking', 'FontSize', 12);

subplot(3,1,2);
plot(t_angles(1:length(ref_angles_total(:,2))), ref_angles_total(:,2), '--b', 'LineWidth', 2);
hold on;
plot(t_angles(1:length(states_total(:,11))), states_total(:,11), 'r', 'LineWidth', 1.5);
grid on;
xlabel('time [s]', 'FontSize', 12);
ylabel('theta [rad]', 'FontSize', 12);
legend({'theta- ref', 'theta- angle'}, 'Location', 'northeast', 'FontSize', 11);
title('Pitch angle tracking', 'FontSize', 12);

subplot(3,1,3);
plot(t_angles(1:length(ref_angles_total(:,3))), ref_angles_total(:,3), '--b', 'LineWidth', 2);
hold on;
plot(t_angles(1:length(states_total(:,12))), states_total(:,12), 'r', 'LineWidth', 1.5);
grid on;
xlabel('time [s]', 'FontSize', 12);
ylabel('psi [rad]', 'FontSize', 12);
legend({'psi- ref', 'psi- angle'}, 'Location', 'northeast', 'FontSize', 11);
title('Yaw angle tracking', 'FontSize', 12);

% FIGURE 6: Control inputs U1, U2, U3, U4 (like Figure A.6)
figure('Position', [100, 100, 800, 600]);
subplot(4,1,1);
plot(t_angles(1:length(UTotal(:,1))), UTotal(:,1), 'k', 'LineWidth', 1);
grid on;
xlabel('time [s]', 'FontSize', 12);
ylabel('U1 [N]', 'FontSize', 12);
title('Thrust input', 'FontSize', 12);

subplot(4,1,2);
plot(t_angles(1:length(UTotal(:,2))), UTotal(:,2), 'k', 'LineWidth', 1);
grid on;
xlabel('time [s]', 'FontSize', 12);
ylabel('U2 [Nm]', 'FontSize', 12);
title('Roll moment', 'FontSize', 12);

subplot(4,1,3);
plot(t_angles(1:length(UTotal(:,3))), UTotal(:,3), 'k', 'LineWidth', 1);
grid on;
xlabel('time [s]', 'FontSize', 12);
ylabel('U3 [Nm]', 'FontSize', 12);
title('Pitch moment', 'FontSize', 12);

subplot(4,1,4);
plot(t_angles(1:length(UTotal(:,4))), UTotal(:,4), 'k', 'LineWidth', 1);
grid on;
xlabel('time [s]', 'FontSize', 12);
ylabel('U4 [Nm]', 'FontSize', 12);
title('Yaw moment', 'FontSize', 12);

fprintf('Plotting completed.\n');
fprintf('Simulation finished successfully!\n');