% Kaitlin Dennison
% Joshua Geiser

clear all; close all; clc;

%% 2-1)
%{
    In general the body axes are not the principal axes. Identify principal axes through the
    eigenvector/eigenvalue problem discussed in class and compute the rotation matrix from body to principal
    axes.
%}

I_body = [+0.20502, -0.10457, +0.00000; ...
          -0.10457, +0.23232, +0.00000; ...
          +0.00000, +0.00000, +0.25733];
[A_rot, I_princ] = eig(I_body);

%% 2-4)
%{
    Numerically integrate Euler equations from arbitrary initial conditions (ω<10°/s, ωi≠0). Multiple attitude
    revolutions
%}

% Renaming variables
Ix = I_princ(1,1); Iy = I_princ(2,2); Iz = I_princ(3,3);

% Initial Conditions
wx_i = deg2rad(3); wy_i = deg2rad(2); wz_i = deg2rad(1);
y0 = [wx_i; wy_i; wz_i];

% Sim time parameters
t0 = 0; dt = 0.1; tf = 200; t_sim = [t0:dt:tf]';

% Call numerical integrator
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
[t_out, y_out] = ode45(@(t,y) int_Euler_eqs(t,y,I_princ), t_sim, y0, options);

%% 2-5 and 2-6)
%{
    Plot rotational kinetic energy and momentum ellipsoids in 3D (axis equal) corresponding to chosen initial
    conditions. Verify that semi-axis of ellipsoids corresponds to theoretical values.

    Plot polhode in same 3D plot. Verify that it is the intersection between the ellipsoids.
%}

T = ((wx_i^2 * Ix) + (wy_i^2 * Iy) + (wz_i^2 * Iz)) / 2;
L = sqrt((wx_i^2 * Ix^2) + (wy_i^2 * Iy^2) + (wz_i^2 * Iz^2));

a_energy = sqrt(2*T/Ix); b_energy = sqrt(2*T/Iy); c_energy = sqrt(2*T/Iz);
a_moment = L/Ix; b_moment = L/Iy; c_moment = L/Iz;

[Xe, Ye, Ze] = ellipsoid(0,0,0,a_energy,b_energy,c_energy);
[Xm, Ym, Zm] = ellipsoid(0,0,0,a_moment,b_moment,c_moment);

figure(); hold on; grid on; axis equal;
surf(Xe,Ye,Ze, 'FaceColor', 'blue');
surf(Xm,Ym,Zm, 'FaceColor', 'green');
plot3(y_out(:,1), y_out(:,2), y_out(:,3), 'r', 'LineWidth', 4);
xlabel('c_x (rad/s)'); ylabel('c_y (rad/s)'); zlabel('c_z (rad/s)');
title('Polhode and Energy/Momentum Ellipsoids');
view(3);

%% 2-7)
%{
    Plot polhode in three 2D planes identified by principal axes (axis equal). Verify that shapes of resulting
    conic sections correspond to theory.
%}

figure();
subplot(1,3,1); hold on; grid on; axis equal; 
plot(y_out(:,1), y_out(:,2)); % xy plane
xlabel('X (rad/s)'); ylabel('Y (rad/s)');

subplot(1,3,2); hold on; grid on; axis equal;
plot(y_out(:,1), y_out(:,3)); % xz plane
xlabel('X (rad/s)'); ylabel('Z (rad/s)');

subplot(1,3,3); hold on; grid on; axis equal;
plot(y_out(:,2), y_out(:,3)); % yz plane
xlabel('Y (rad/s)'); ylabel('Z (rad/s)');



% Numerical integration of Euler's equations
function ydot = int_Euler_eqs(t,y,I_princ)

% Renaming variables
Ix = I_princ(1,1); Iy = I_princ(2,2); Iz = I_princ(3,3);
wx = y(1); wy = y(2); wz = y(3);
Mx = 0; My = 0; Mz = 0;

% State space form
ydot = zeros(3,1);
ydot(1) = (Mx - (Iz - Iy) * wy * wz) / Ix;
ydot(2) = (My - (Ix - Iz) * wz * wx) / Iy;
ydot(3) = (Mz - (Iy - Ix) * wx * wy) / Iz;
end