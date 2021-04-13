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
    quiver3(0,0,0,triad_prin(1,1),triad_prin(2,1),triad_prin(3,1),...
        'r--','LineWidth',2,'DisplayName','X-Principle')
    quiver3(0,0,0,triad_prin(1,2),triad_prin(2,2),triad_prin(3,2),...
        'g--','LineWidth',2,'DisplayName','Y-Principle')
    quiver3(0,0,0,triad_prin(1,3),triad_prin(2,3),triad_prin(3,3),...
        'c--','LineWidth',2,'DisplayName','Z-Principle')
    
    quiver3(0,0,0,triad_body(1,1),triad_body(2,1),triad_body(3,1),...
        'r','LineWidth',2,'DisplayName','X-Body')
    quiver3(0,0,0,triad_body(1,2),triad_body(2,2),triad_body(3,2),...
        'g','LineWidth',2,'DisplayName','Y-Body')
    quiver3(0,0,0,triad_body(1,3),triad_body(2,3),triad_body(3,3),...
        'c','LineWidth',2,'DisplayName','Z-Body')
    
    % 3D model
    patch('Faces',f_bus,'Vertices',v_bus,'FaceColor',[0.5,0.5,0.5],...
        'FaceAlpha',0.75,'DisplayName','Main Bus')
    patch('Faces',f_pan,'Vertices',v_pan_1,'FaceColor','blue',...
        'FaceAlpha',0.75,'DisplayName','Panel')
    patch('Faces',f_pan,'Vertices',v_pan_2,'FaceColor','blue',...
        'FaceAlpha',0.75,'DisplayName','Panel')
    
    xlabel('X (cm)')
    ylabel('Y (cm)')
    zlabel('Z (cm)')
    view(80+90,50)
    axis equal
    legend()
    
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
w0 = deg2rad([-3;2;1]);
norm(rad2deg(w0))

% Sim time parameters
t0 = 0; dt = 0.1; tf = 500; t_sim = (t0:dt:tf)';

% Call numerical integrator
options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
[t_out, y_out] = ode45(@(t,y) int_Euler_eqs(t,y,I_princ), t_sim, w0, options);

%% 2-5 and 2-6)
%{
    5. Plot rotational kinetic energy and momentum ellipsoids in 3D (axis
    equal) corresponding to chosen initial conditions. Verify that
    semi-axis of ellipsoids corresponds to theoretical values.

    6. Plot polhode in same 3D plot. Verify that it is the intersection
    between the ellipsoids.

    7. Plot polhode in three 2D planes identified by principal axes (axis
    equal). Verify that shapes of resulting conic sections correspond to
    theory.
%}

[x_pol,y_pol] = polhodeAnalytical(I_princ,w0);
PlotPolhode(I_princ,w0,y_out,x_pol,y_pol)

%% 2-8)
%{
    Repeat above steps changing initial conditions, e.g. setting angular
    velocity vector parallel to principal axis. Is the behavior according
    to expectations?
%}

% Initial Conditions
w0_new = A_rot*deg2rad([3;3;3]);
norm(rad2deg(w0_new))

% Simulate
[~, y_new] = ode45(@(t,y) int_Euler_eqs(t,y,I_princ), t_sim, w0_new, options);
[x_pnew,y_pnew] = polhodeAnalytical(I_princ,w0_new);
PlotPolhode(I_princ,w0_new,y_new,x_pnew,y_pnew)

%% 2-9)
%{
    Impose that satellite is axial-symmetric (Ix=Iy≠Iz). Repeat numerical
    simulation using initial condition 4).

%}

% Setting the satellite to be axially-symmetric about the x-axis
Ix = (I_princ(1,1)+I_princ(2,2))/2;
Iy = Ix;
I_symm = diag([Ix,Iy,Iz]);

% Simulate
[~, w_sy] = ode45(@(t,y) int_Euler_eqs(t,y,I_symm), t_sim, w0, options);

%% 2-10)
%{
    Program analytical solution for axial-symmetric satellite. Compute
    it at same time steps and from same initial conditions as 9).
%}

[x_psymm,y_psymm,w_an] = polhodeAnalytical(I_symm,w0,t_sim);
PlotPolhode(I_symm,w0,w_sy)

figure();

    % xy plane
    subplot(1,3,1); hold on; grid on; axis equal; 
    plot(w_sy(:,1), w_sy(:,2), 'LineWidth', 2); 
    xlabel('\omega_x (rad/s)'); ylabel('\omega_y (rad/s)');
    title('Polhode in XY Plane');
    plot(w_an(:,1),w_an(:,2), 'r-.');

    % xz plane
    subplot(1,3,2); hold on; grid on; axis equal;
    plot(w_sy(:,1), w_sy(:,3), 'LineWidth', 2); 
    xlabel('\omega_x (rad/s)'); ylabel('\omega_z (rad/s)');
    title('Polhode in XZ Plane');
    plot(w_an(:,1),w_an(:,3),'r-.');

    % yz plane
    subplot(1,3,3); hold on; grid on; axis equal;
    plot(w_sy(:,2), w_sy(:,3), 'LineWidth', 2); 
    xlabel('\omega_y (rad/s)'); ylabel('\omega_z (rad/s)');
    title('Polhode in YZ Plane');
    plot(w_an(:,2),w_an(:,3), 'r-.');
    legend('Numerical', 'Analytical');

%% 2-11) 
%{
    Compare numerical and analytical solutions. Plot differences (errors),
    do not only superimpose absolute values. Tune numerical integrator for
    large discrepancies. Are angular velocity vector and angular momentum
    vector changing according to theory in principal axes?
%}

figure();
    
    subplot(3,1,1); hold on
    title('Error between Analytical and Numerical')
    plot(t_sim,w_sy(:,1)-w_an(:,1))
    xlabel('time (s)')
    ylabel('\delta\omega_x (rad/s)')
    subplot(3,1,2); hold on
    plot(t_sim,w_sy(:,2)-w_an(:,2))
    xlabel('time (s)')
    ylabel('\delta\omega_y (rad/s)')
    subplot(3,1,3); hold on
    plot(t_sim,w_sy(:,3)-w_an(:,3))
    xlabel('time (s)')
    ylabel('\delta\omega_z (rad/s)')

%% Extra Function

function PlotPolhode(I,w0,y_out,x_pol,y_pol)

Ix = I(1,1);
Iy = I(2,2);
Iz = I(3,3);

% Ellipse Parameters
T = ((w0(1)^2 * Ix) + (w0(2)^2 * Iy) + (w0(3)^2 * Iz)) / 2;
L = sqrt((w0(1)^2 * Ix^2) + (w0(2)^2 * Iy^2) + (w0(3)^2 * Iz^2));

a_energy = sqrt(2*T/Ix);
b_energy = sqrt(2*T/Iy);
c_energy = sqrt(2*T/Iz);
a_moment = L/Ix;
b_moment = L/Iy;
c_moment = L/Iz;

[Xe, Ye, Ze] = ellipsoid(0,0,0,a_energy,b_energy,c_energy);
[Xm, Ym, Zm] = ellipsoid(0,0,0,a_moment,b_moment,c_moment);

% Energy and Momentum Ellipsoids in 3D
figure(); 
    
    hold on; grid on; axis equal;
    surf(Xe,Ye,Ze, 'FaceColor', 'blue', 'DisplayName', 'Energy Ellipsoid');
    surf(Xm,Ym,Zm, 'FaceColor', 'green', 'DisplayName', 'Momentum Ellipsoid');
    plot3(y_out(:,1), y_out(:,2), y_out(:,3), 'r', 'LineWidth', 4, 'DisplayName', 'Polhode');
    xlabel('\omega_x (rad/s)'); ylabel('\omega_y (rad/s)'); zlabel('\omega_z (rad/s)');
    title('Polhode and Energy/Momentum Ellipsoids'); legend();
    view(3);
   
% Energy and Momentum Ellipsoids 2D views
%{
figure();
    
    % xy plane
    subplot(1,3,1); hold on; grid on; axis equal;
    surf(Xe,Ye,Ze, 'FaceColor', 'blue', 'DisplayName', 'Energy Ellipsoid');
    surf(Xm,Ym,Zm, 'FaceColor', 'green', 'DisplayName', 'Momentum Ellipsoid');
    plot3(y_out(:,1), y_out(:,2), y_out(:,3), 'r', 'LineWidth', 4, 'DisplayName', 'Polhode');
    xlabel('\omega_x (rad/s)'); ylabel('\omega_y (rad/s)'); zlabel('\omega_z (rad/s)');
    title('Polhode and Energy/Momentum Ellipsoids in XY'); legend();
    view([0,0,1]);
    
    % xz plane
    subplot(1,3,2); hold on; grid on; axis equal;
    surf(Xe,Ye,Ze, 'FaceColor', 'blue', 'DisplayName', 'Energy Ellipsoid');
    surf(Xm,Ym,Zm, 'FaceColor', 'green', 'DisplayName', 'Momentum Ellipsoid');
    plot3(y_out(:,1), y_out(:,2), y_out(:,3), 'r', 'LineWidth', 4, 'DisplayName', 'Polhode');
    xlabel('\omega_x (rad/s)'); ylabel('\omega_y (rad/s)'); zlabel('\omega_z (rad/s)');
    title('Polhode and Energy/Momentum Ellipsoids in XZ'); legend();
    view([0,-1,0]);
    
    % yz plane
    subplot(1,3,3); hold on; grid on; axis equal;
    surf(Xe,Ye,Ze, 'FaceColor', 'blue', 'DisplayName', 'Energy Ellipsoid');
    surf(Xm,Ym,Zm, 'FaceColor', 'green', 'DisplayName', 'Momentum Ellipsoid');
    plot3(y_out(:,1), y_out(:,2), y_out(:,3), 'r', 'LineWidth', 4, 'DisplayName', 'Polhode');
    xlabel('\omega_x (rad/s)'); ylabel('\omega_y (rad/s)'); zlabel('\omega_z (rad/s)');
    title('Polhode and Energy/Momentum Ellipsoids in YZ'); legend();
    view([1,0,0]);
%}

if exist('x_pol','var')

    figure();

        % xy plane
        subplot(1,3,1); hold on; grid on; axis equal; 
        plot(y_out(:,1), y_out(:,2), 'LineWidth', 2); 
        xlabel('\omega_x (rad/s)'); ylabel('\omega_y (rad/s)');
        title('Polhode in XY Plane');
        plot(x_pol(:,1),y_pol(:,1), 'r-.');

        % xz plane
        subplot(1,3,2); hold on; grid on; axis equal;
        plot(y_out(:,1), y_out(:,3), 'LineWidth', 2); 
        xlabel('\omega_x (rad/s)'); ylabel('\omega_z (rad/s)');
        title('Polhode in XZ Plane');
        plot(x_pol(:,3), y_pol(:,3), 'r-.', x_pol(:,4), y_pol(:,4), 'r-.', x_pol(:,5), y_pol(:,5), 'r-.', x_pol(:,6), y_pol(:,6), 'r-.');

        % yz plane
        subplot(1,3,3); hold on; grid on; axis equal;
        plot(y_out(:,2), y_out(:,3), 'LineWidth', 2); 
        xlabel('\omega_y (rad/s)'); ylabel('\omega_z (rad/s)');
        title('Polhode in YZ Plane');
        plot(x_pol(:,2),y_pol(:,2), 'r-.');
        legend('Numerical', 'Analytical');

end
    
end