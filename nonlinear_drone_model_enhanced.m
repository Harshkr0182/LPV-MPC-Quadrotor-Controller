function dx = nonlinear_drone_model_enhanced(t, states, U, Ix, Iy, Iz, m, g, Jtp, omega_total)
%% Enhanced nonlinear drone model with better numerical stability

% Extract states with bounds for numerical stability
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
theta = max(min(states(11), pi/2 - 0.01), -pi/2 + 0.01);  % Avoid singularity
psi = states(12);

% Inputs
U1 = U(1);
U2 = U(2);
U3 = U(3);
U4 = U(4);

%% Rotation matrix with protection
cos_theta = cos(theta);
sin_theta = sin(theta);
cos_phi = cos(phi);
sin_phi = sin(phi);
cos_psi = cos(psi);
sin_psi = sin(psi);

R_matrix = [cos_theta*cos_psi, sin_phi*sin_theta*cos_psi - cos_phi*sin_psi, ...
            cos_phi*sin_theta*cos_psi + sin_phi*sin_psi; ...
            cos_theta*sin_psi, sin_phi*sin_theta*sin_psi + cos_phi*cos_psi, ...
            cos_phi*sin_theta*sin_psi - sin_phi*cos_psi; ...
            -sin_theta, sin_phi*cos_theta, cos_phi*cos_theta];

%% Transformation matrix with protection
if abs(cos_theta) < 1e-6
    cos_theta = sign(cos_theta) * 1e-6;
end
sec_theta = 1/cos_theta;
tan_theta = sin_theta/cos_theta;

T_matrix = [1, sin_phi*tan_theta, cos_phi*tan_theta; ...
            0, cos_phi, -sin_phi; ...
            0, sin_phi*sec_theta, cos_phi*sec_theta];

%% Nonlinear dynamics with rate limiting for stability
% Body frame velocities
dx1 = (v*r - w*q) + g*sin_theta;                           % u_dot
dx2 = (w*p - u*r) - g*cos_theta*sin_phi;                   % v_dot
dx3 = (u*q - v*p) - g*cos_theta*cos_phi + U1/m;            % w_dot

% Body frame angular velocities
dx4 = q*r*(Iy-Iz)/Ix - Jtp/Ix*q*omega_total + U2/Ix;       % p_dot
dx5 = p*r*(Iz-Ix)/Iy + Jtp/Iy*p*omega_total + U3/Iy;       % q_dot
dx6 = p*q*(Ix-Iy)/Iz + U4/Iz;                              % r_dot

% Earth frame positions
earth_vel = R_matrix * [u; v; w];
dx7 = earth_vel(1);  % x_dot
dx8 = earth_vel(2);  % y_dot
dx9 = earth_vel(3);  % z_dot

% Earth frame angles
earth_ang_vel = T_matrix * [p; q; r];
dx10 = earth_ang_vel(1);  % phi_dot
dx11 = earth_ang_vel(2);  % theta_dot
dx12 = earth_ang_vel(3);  % psi_dot

dx = [dx1; dx2; dx3; dx4; dx5; dx6; dx7; dx8; dx9; dx10; dx11; dx12];

end