function [Phi_ref, Theta_ref, U1] = pos_controller(...
    X_ref, X_dot_ref, Y_ref, Y_dot_ref, Z_ref, Z_dot_ref, Psi_ref, ...
    states, m, g, px, py, pz)
%% Position controller using feedback linearization
% Based on equations (3.24) to (3.33) in the thesis

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

% Rotation matrix (Equation 2.6)
R_matrix = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
            cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); ...
            cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), ...
            cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); ...
            -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];

% Current velocities in earth frame
x_dot = R_matrix(1,:) * [u; v; w];
y_dot = R_matrix(2,:) * [u; v; w];
z_dot = R_matrix(3,:) * [u; v; w];

%% Compute errors (Equation 3.27)
ex = X_ref - x;
ex_dot = X_dot_ref - x_dot;
ey = Y_ref - y;
ey_dot = Y_dot_ref - y_dot;
ez = Z_ref - z;
ez_dot = Z_dot_ref - z_dot;

%% Compute feedback gains using pole placement
% x-direction
Ax = [0 1; 0 0];
Bx = [0; 1];
Kx = place(Ax, Bx, px);
ux = -Kx * [ex; ex_dot];
vx = -ux;

% y-direction
Ay = [0 1; 0 0];
By = [0; 1];
Ky = place(Ay, By, py);
uy = -Ky * [ey; ey_dot];
vy = -uy;

% z-direction
Az = [0 1; 0 0];
Bz = [0; 1];
Kz = place(Az, Bz, pz);
uz = -Kz * [ez; ez_dot];
vz = -uz;

%% Compute desired angles and thrust (Equations 3.30-3.33)
a = vx / (vz + g);
b = vy / (vz + g);
c = cos(Psi_ref);
d = sin(Psi_ref);

% Theta (pitch) - Equation 3.30
tan_theta = a*c + b*d;
Theta_ref = atan(tan_theta);

% Phi (roll) - Equation 3.31
if abs(Psi_ref) < pi/4 || abs(Psi_ref) > 3*pi/4
    tan_phi = cos(Theta_ref) * (tan(Theta_ref)*d - b) / c;
else
    tan_phi = cos(Theta_ref) * (a - tan(Theta_ref)*c) / d;
end
Phi_ref = atan(tan_phi);

% Thrust U1 - Equation 3.33
U1 = (vz + g) * m / (cos(Phi_ref) * cos(Theta_ref));

end