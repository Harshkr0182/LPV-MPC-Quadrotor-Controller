function [Phi_ref, Theta_ref, U1, ex_integral_new, ey_integral_new, ez_integral_new] = ...
    pos_controller_enhanced(...
    X_ref, X_dot_ref, Y_ref, Y_dot_ref, Z_ref, Z_dot_ref, Psi_ref, ...
    states, m, g, px, py, pz, ex_integral, ey_integral, ez_integral, dt)
%% Enhanced position controller with integral action for better tracking

% Integral gains
Ki_x = 0.5;
Ki_y = 0.5;
Ki_z = 0.5;

% Extract states
u = states(1);
v = states(2);
w = states(3);
x = states(7);
y = states(8);
z = states(9);
phi = states(10);
theta = states(11);
psi = states(12);

% Rotation matrix
R_matrix = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
            cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); ...
            cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), ...
            cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); ...
            -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];

% Current velocities
x_dot = R_matrix(1,:) * [u; v; w];
y_dot = R_matrix(2,:) * [u; v; w];
z_dot = R_matrix(3,:) * [u; v; w];

%% Compute errors with integral
ex = X_ref - x;
ex_dot = X_dot_ref - x_dot;
ex_integral_new = ex_integral + ex * dt;

ey = Y_ref - y;
ey_dot = Y_dot_ref - y_dot;
ey_integral_new = ey_integral + ey * dt;

ez = Z_ref - z;
ez_dot = Z_dot_ref - z_dot;
ez_integral_new = ez_integral + ez * dt;

%% Compute feedback gains
% x-direction with integral
Ax = [0 1 0; 0 0 0; 0 0 0];
Bx = [0; 1; 0];
% Using PD control (integral handled separately)
Kx = place([0 1; 0 0], [0; 1], px);
ux = -Kx * [ex; ex_dot] - Ki_x * ex_integral_new;
vx = -ux;

% y-direction
Ky = place([0 1; 0 0], [0; 1], py);
uy = -Ky * [ey; ey_dot] - Ki_y * ey_integral_new;
vy = -uy;

% z-direction
Kz = place([0 1; 0 0], [0; 1], pz);
uz = -Kz * [ez; ez_dot] - Ki_z * ez_integral_new;
vz = -uz;

%% Compute desired angles and thrust
a = vx / (vz + g + 1e-6);  % Add small epsilon to avoid division by zero
b = vy / (vz + g + 1e-6);
c = cos(Psi_ref);
d = sin(Psi_ref);

% Theta (pitch)
tan_theta = a*c + b*d;
Theta_ref = atan(tan_theta);

% Phi (roll) with numerical protection
if abs(Psi_ref) < pi/4 || abs(Psi_ref) > 3*pi/4
    denominator = c;
else
    denominator = d;
end

if abs(denominator) < 1e-6
    denominator = sign(denominator) * 1e-6;
end

if abs(Psi_ref) < pi/4 || abs(Psi_ref) > 3*pi/4
    tan_phi = cos(Theta_ref) * (tan(Theta_ref)*d - b) / denominator;
else
    tan_phi = cos(Theta_ref) * (a - tan(Theta_ref)*c) / denominator;
end

% Limit tan_phi to avoid extreme angles
tan_phi = max(min(tan_phi, 5), -5);
Phi_ref = atan(tan_phi);

% Thrust U1 with saturation protection
U1 = (vz + g) * m / (cos(Phi_ref) * cos(Theta_ref) + 1e-6);

end