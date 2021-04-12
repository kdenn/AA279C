% Kaitlin Dennison
% Joshua Geiser

clear all; close all; clc;

%% 2-1)
%{
    In general the body axes are not the principal axes. Identify principal
    axes through the eigenvector/eigenvalue problem discussed in class and
    compute the rotation matrix from body to principal axes.
%}

I_body = [+0.20502, -0.10457, +0.00000; ...
          -0.10457, +0.23232, +0.00000; ...
          +0.00000, +0.00000, +0.25733];
[A_rot, I_princ] = eig(I_body);

%% 2-2) 
%{
    At this stage you should have a simple 3D model of your spacecraft
    including geometry and mass properties of each element. This includes
    at least two coordinate systems, body and principal axes respectively,
    and the direction cosine matrix between them. Plot axes of triads in 3D
    superimposed to spacecraft 3D model.
%}

visorsModel;
% f_bus, v_bus, f_pan, v_pan_1, v_pan_2

triad_body = 30.*eye(3);
triad_prin = A_rot*triad_body;

figure(); hold on
    
    % triad 
    quiver3(0,0,0,triad_prin(1,1),triad_prin(2,1),triad_prin(3,1),'r--')
    quiver3(0,0,0,triad_prin(1,2),triad_prin(2,2),triad_prin(3,2),'g--')
    quiver3(0,0,0,triad_prin(1,3),triad_prin(2,3),triad_prin(3,3),'c--')
    
    quiver3(0,0,0,triad_body(1,1),triad_body(2,1),triad_body(3,1),'r')
    quiver3(0,0,0,triad_body(1,2),triad_body(2,2),triad_body(3,2),'g')
    quiver3(0,0,0,triad_body(1,3),triad_body(2,3),triad_body(3,3),'c')
    
    % 3D model
    patch('Faces',f_bus,'Vertices',v_bus,'FaceColor',[0.5,0.5,0.5],'FaceAlpha',0.75)
    patch('Faces',f_pan,'Vertices',v_pan_1,'FaceColor','blue','FaceAlpha',0.75)
    patch('Faces',f_pan,'Vertices',v_pan_2,'FaceColor','blue','FaceAlpha',0.75)
    
    xlabel('X (cm)')
    ylabel('Y (cm)')
    zlabel('Z (cm)')
    view(80+90,50)
    axis equal
    
    legend('X-Prin','Y-Prin','Z-Prin','X-Body','Y-Body','Z-Body')
    
hold off

%% 2-3)
%{
    Program Euler equations in principal axes (e.g. in Matlab/Simulink). No
    external torques.
%}

% See propagation\int_Euler_eqs.m

%% 2-4)
%{
    Numerically integrate Euler equations from arbitrary initial conditions
    (ω<10°/s, ωi≠0). Multiple attitude revolutions
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
    5. Plot rotational kinetic energy and momentum ellipsoids in 3D (axis
    equal) corresponding to chosen initial conditions. Verify that
    semi-axis of ellipsoids corresponds to theoretical values.

    6. Plot polhode in same 3D plot. Verify that it is the intersection
    between the ellipsoids.
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
    Plot polhode in three 2D planes identified by principal axes (axis
    equal). Verify that shapes of resulting conic sections correspond to
    theory.
%}

figure();

% xy plane
subplot(1,3,1); hold on; grid on; axis equal; 
plot(y_out(:,1), y_out(:,2), 'LineWidth', 3); 
xlabel('c_x (rad/s)'); ylabel('c_y (rad/s)');
title('Polhode in XY Plane');
a = sqrt( (L^2-2*T*Iz) / ((Ix-Iz)*Ix) );
b = sqrt( (L^2-2*T*Iz) / ((Iy-Iz)*Iy) );
t = linspace(0,2*pi); x = a*cos(t); y = b*sin(t);
plot(x,y, 'r-.');

% xz plane
subplot(1,3,2); hold on; grid on; axis equal;
plot(y_out(:,1), y_out(:,3), 'LineWidth', 3); 
xlabel('c_x (rad/s)'); ylabel('c_z (rad/s)');
title('Polhode in XZ Plane');
a = - sqrt( (L^2-2*T*Iy) / ((Ix-Iy)*Ix) );
b = sqrt( -(L^2-2*T*Iy) / ((Iz-Iy)*Iz) );
e = sqrt(1 + (b^2/abs(a^2)));
x = linspace(a, 2*a, 100);
ytop = b.*sqrt(x.^2 ./ a.^2 - 1);
ybot = -b.*sqrt(x.^2 ./ a.^2 - 1);
plot(x, ytop, 'r-.', x, ybot, 'r-.', -x, ytop, 'r-.', -x, ybot, 'r-.');

% yz plane
subplot(1,3,3); hold on; grid on; axis equal;
plot(y_out(:,2), y_out(:,3), 'LineWidth', 3); 
xlabel('c_y (rad/s)'); ylabel('c_z (rad/s)');
title('Polhode in YZ Plane');
a = sqrt( (L^2-2*T*Ix) / ((Iy-Ix)*Iy) );
b = sqrt( (L^2-2*T*Ix) / ((Iz-Ix)*Iz) );
t = linspace(0,2*pi); x = a*cos(t); y = b*sin(t);
plot(x,y, 'r-.');
legend('Numerical', 'Analytical');

%% 2-8)
%{
    Repeat above steps changing initial conditions, e.g. setting angular
    velocity vector parallel to principal axis. Is the behavior according
    to expectations?
%}

%TODO

%% 2-9)
%{
    Impose that satellite is axial-symmetric (Ix=Iy≠Iz). Repeat numerical
    simulation using initial condition 4).
%}

%TODO

%% 2-10) 
%{
    Program analytical solution for axial-symmetric satellite. Compute it
    at same time steps and from same initial conditions as 9).
%}

%TODO

%% 2-11) 
%{
    Compare numerical and analytical solutions. Plot differences (errors),
    do not only superimpose absolute values. Tune numerical integrator for
    large discrepancies. Are angular velocity vector and angular momentum
    vector changing according to theory in principal axes?
%}

%TODO