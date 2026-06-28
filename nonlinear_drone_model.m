function dx = nonlinear_drone_model(t, states, U, Ix, Iy, Iz, m, g, Jtp, omega_total)
%% Nonlinear drone dynamics (Equation 2.8)
% This is the open-loop plant model

% Extract states
u = states(1);
v = states(2);
w = states(3);
p = states(4);
q = states(5);
r = states(6);
x = states(7);
y = states(8);
z = states(9);
phi = states(10);
theta = states(11);
psi = states(12);

% Inputs
U1 = U(1);
U2 = U(2);
U3 = U(3);
U4 = U(4);

%% Rotation matrix (Equation 2.6)
R_matrix = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
            cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); ...
            cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), ...
            cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); ...
            -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];

%% Transformation matrix for angular velocities (Equation 2.7)
T_matrix = [1, sin(phi)*tan(theta), cos(phi)*tan(theta); ...
            0, cos(phi), -sin(phi); ...
            0, sin(phi)*sec(theta), cos(phi)*sec(theta)];

%% Nonlinear dynamics (Equation 2.8)
dx = zeros(12, 1);

% Body frame velocities
dx(1) = (v*r - w*q) + g*sin(theta);                        % u_dot
dx(2) = (w*p - u*r) - g*cos(theta)*sin(phi);               % v_dot
dx(3) = (u*q - v*p) - g*cos(theta)*cos(phi) + U1/m;        % w_dot

% Body frame angular velocities
dx(4) = q*r*(Iy-Iz)/Ix - Jtp/Ix*q*omega_total + U2/Ix;     % p_dot
dx(5) = p*r*(Iz-Ix)/Iy + Jtp/Iy*p*omega_total + U3/Iy;     % q_dot
dx(6) = p*q*(Ix-Iy)/Iz + U4/Iz;                            % r_dot

% Earth frame positions
dx(7:9) = R_matrix * [u; v; w];                            % x_dot, y_dot, z_dot

% Earth frame angles
dx(10:12) = T_matrix * [p; q; r];                          % phi_dot, theta_dot, psi_dot

end