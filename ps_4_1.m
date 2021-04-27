% Kaitlin Dennison
% Joshua Geiser

clear all; close all; clc;

%% 1) Adding a momentum wheel or rotor (dual-spin satellite)

visors = visorsStruct();

% Initial Conditions
wz0 = deg2rad(1);
wr0 = deg2rad(360);
w0 = [0;wz0;0;wr0]; % 1a-c: [0;0;wz0;wr0];  1d: [0;wz0;0;wr0];
q0 = [0; 0; 0; 1];

% Rotor initial conditions
Ir = 3 * (0.5 * .13 * 0.042^2); % assuming 3 reaction wheels
r_rot = [0; 1; 0]; % 1a-c: [0;0;1];  1d: [0;1;0];

% Sim time parameters
t0 = 0; dt = 0.5; tf = 60*45; t_arr = (t0:dt:tf)';

% Propagate angular velocity and attitude
[w_body, quat_out] = propagate_attitude_rotor(t_arr, w0, q0, Ir, r_rot);

%% 1b
%{
Numerically integrate Euler AND Kinematic equations from equilibrium initial condition. Verify
that integration is correct as from previous tests (conservation laws, rotations, etc.)
%}

% Calculations for inertial angular momentum/velocity
L_body = visors.I_princ * w_body(1:3,:) + Ir*r_rot*w_body(4,:);
L_inert = zeros(3,length(t_arr));
w_inert = zeros(3,length(t_arr));
for i = 1:length(t_arr)
    DCM = quat2dcm(quat_out(:,i));
    L_inert(:,i) = DCM' * L_body(:,i);
    w_inert(:,i) = DCM' * w_body(1:3,i);
end

figure();
subplot(3,1,1); hold on; grid on;
plot(t_arr'./60, L_inert(1,:));
xlabel('Time (min)'); ylabel('L_x (kg m^2 / s)');
axis([0 tf/60 -0.01 0.01]);
title('Inertial Angular Momentum vs Time - L_x');

subplot(3,1,2); hold on; grid on;
plot(t_arr'./60, L_inert(2,:));
xlabel('Time (min)'); ylabel('L_y (kg m^2 / s)');
axis([0 tf/60 -0.01 0.01]);
title('Inertial Angular Momentum vs Time - L_y');

subplot(3,1,3); hold on; grid on;
plot(t_arr'./60, L_inert(3,:));
xlabel('Time (min)'); ylabel('L_z (kg m^2 / s)');
axis([0 tf/60 -0.01 0.01]);
title('Inertial Angular Momentum vs Time - L_z');

%% Equilibrium Tests
t_arr_min = t_arr./60;

    % Angular Velocity
    figure(); hold on; grid on;

        plot(t_arr_min,w_body(1,:),'DisplayName','\omega_x ','LineWidth',2,'LineStyle',':')
        plot(t_arr_min,w_body(2,:),'DisplayName','\omega_y ')
        plot(t_arr_min,w_body(3,:),'DisplayName','\omega_z ')
        xlabel('t (min)')
        ylabel('\omega (rad/s)')
        axis([0,12,-0.03,0.03])
        legend()

    hold off
    
    % Quaternions
    figure(); hold on; grid on;

        plot(t_arr_min,quat_out(1,:),'DisplayName','q_1','LineWidth',2,'LineStyle',':')
        plot(t_arr_min,quat_out(2,:),'DisplayName','q_2')
        plot(t_arr_min,quat_out(3,:),'DisplayName','q_3')
        plot(t_arr_min,quat_out(4,:),'DisplayName','q_4')
        xlabel('t (min)')
        ylabel('q')
        axis([0,12,-1,1])
        legend()

    hold off

    % Euler Angles
    A = quat2dcm(quat_out);
    eulax = dcm2eulax(A);
    figure(); hold on

        subplot(2,1,1); hold on; grid on;
        plot(t_arr_min,eulax(1,:),'DisplayName','e_x','LineWidth',2,'LineStyle',':')
        plot(t_arr_min,eulax(2,:),'DisplayName','e_y')
        plot(t_arr_min,eulax(3,:),'DisplayName','e_z')
        xlabel('t (min)')
        ylabel('e')
        axis([0,12,-1,1])
        legend()
        hold off

        subplot(2,1,2); hold on; grid on;
        plot(t_arr_min,eulax(4,:),'DisplayName','\Phi')
        xlabel('t (min)')
        ylabel('\Phi (rad)')
        axis([0,12,0,pi])
        hold off

    hold off
    
%% Stability Tests

w0_x = [wz0;0;0;wr0]; % rotation only about inertial x
w0_y = [0;wz0;0;wr0]; % rotation only about inertial y
w0_z = [0;0;wz0;wr0]; % rotation only about inertial z
qns_eq = [0;0;0;1]; % = dcm2quat(eye(3)); % quaterions for all eul ang = 0

w_pert = -wz0.*[.01;.01;.01;0];

[w_out_x, qns_out_x] = propagate_attitude_rotor(t_arr, w0_x+w_pert, qns_eq, Ir, r_rot);
[w_out_y, qns_out_y] = propagate_attitude_rotor(t_arr, w0_y+w_pert, qns_eq, Ir, r_rot);
[w_out_z, qns_out_z] = propagate_attitude_rotor(t_arr, w0_z+w_pert, qns_eq, Ir, r_rot);

figure(); hold on; grid on;
plot(t_arr_min,w_out_x(1,:),'DisplayName','\omega_x')
plot(t_arr_min,w_out_x(2,:),'DisplayName','\omega_y','LineWidth',2,'LineStyle',':')
plot(t_arr_min,w_out_x(3,:),'DisplayName','\omega_z')
xlabel('t (min)')
ylabel('\omega (rad/s)')
axis([0,12*4,-0.03,0.03])
legend()
%title('X-Spin')
hold off

figure(); hold on; grid on;
plot(t_arr_min,w_out_y(1,:),'DisplayName','\omega_x','LineWidth',2,'LineStyle',':')
plot(t_arr_min,w_out_y(2,:),'DisplayName','\omega_y')
plot(t_arr_min,w_out_y(3,:),'DisplayName','\omega_z')
xlabel('t (min)')
ylabel('\omega (rad/s)')
axis([0,12*4,-0.03,0.03])
legend()
%title('Y-Spin')
hold off

figure(); hold on; grid on;
plot(t_arr_min,w_out_z(1,:),'DisplayName','\omega_x','LineWidth',2,'LineStyle',':')
plot(t_arr_min,w_out_z(2,:),'DisplayName','\omega_y')
plot(t_arr_min,w_out_z(3,:),'DisplayName','\omega_z')
xlabel('t (min)')
ylabel('\omega (rad/s)')
axis([0,12*4,-0.03,0.03])
legend()
%title('Z-Spin')
hold off


%% 1e) Make rotation about another axis stable (Sun-pointing body X axis)

visors = visorsStruct();

% Initial Conditions
%wz0 = deg2rad(1);
wr0 = deg2rad(360);
w0 = [-0.751477370342678*visors.n; -0.659758866452625*visors.n; 0; wr0];
q0 = dcm2quat(visors.A_rot); % q0 = [0; 0; 0; 1];

% Rotor initial conditions
Ir = 3 * (0.5 * .13 * 0.042^2); % assuming 3 reaction wheels
r_rot = [-0.751477370342678; -0.659758866452625; 0]; % 1a-c: [0;0;1];  1d: [0;1;0];

% Sim time parameters
t0 = 0; dt = 0.5; tf = visors.T*2*pi; t_arr = (t0:dt:tf)';

% Propagate angular velocity and attitude
w_perturb = (visors.n / 10) .* [1;1;1;0];
[w_princ, quat_out] = propagate_attitude_rotor(t_arr, w0+w_perturb, q0, Ir, r_rot);

w_body = zeros(4, length(w_princ));
w_body(4,:) = w_princ(4,:);
for i = 1:length(w_princ)
    w_body(1:3,i) = visors.A_rot' *  w_princ(1:3,i);
end

plot_attitude_3D(visors, quat_out);

function return_flag = plot_attitude_3D(visors, quat_out)

triad_inert = 1.*eye(3);
x_prin = zeros(3,length(quat_out));
y_prin = zeros(3,length(quat_out));
z_prin = zeros(3,length(quat_out));
x_body = zeros(3,length(quat_out));
y_body = zeros(3,length(quat_out));
z_body = zeros(3,length(quat_out));
for i = 1:length(quat_out)
    DCM = quat2dcm(quat_out(:,i));
    triad_prin = DCM * triad_inert;
    triad_body = visors.A_rot' * triad_prin;
    
    x_prin(:,i) = triad_prin(:,1);
    y_prin(:,i) = triad_prin(:,2);
    z_prin(:,i) = triad_prin(:,3);
    x_body(:,i) = triad_body(:,1);
    y_body(:,i) = triad_body(:,2);
    z_body(:,i) = triad_body(:,3);
end

figure(); hold on; grid on;
plot3(x_prin(1,:), x_prin(2,:), x_prin(3,:), 'r:', 'DisplayName', 'X Axis', 'LineWidth', 2);
plot3(y_prin(1,:), y_prin(2,:), y_prin(3,:), 'g-.', 'DisplayName', 'Y Axis');
plot3(z_prin(1,:), z_prin(2,:), z_prin(3,:), 'b-.', 'DisplayName', 'Z Axis');
quiver3(0,0,0,x_prin(1,1), x_prin(2,1), x_prin(3,1), 'r', 'AutoScaleFactor', 1, 'DisplayName', 'X Initial');
quiver3(0,0,0,y_prin(1,1), y_prin(2,1), y_prin(3,1), 'g', 'AutoScaleFactor', 1, 'DisplayName', 'Y Initial');
quiver3(0,0,0,z_prin(1,1), z_prin(2,1), z_prin(3,1), 'b', 'AutoScaleFactor', 1, 'DisplayName', 'Z Initial');
xlabel('Inertial I'); ylabel('Inertial J'); zlabel('Inertial K');
title('Orientation of Principal Axes (wrt Inertial) over Time');
view(3); legend(); axis equal;

figure(); hold on; grid on;
plot3(x_body(1,:), x_body(2,:), x_body(3,:), 'r:', 'DisplayName', 'X Axis', 'LineWidth', 2);
plot3(y_body(1,:), y_body(2,:), y_body(3,:), 'g-.', 'DisplayName', 'Y Axis');
plot3(z_body(1,:), z_body(2,:), z_body(3,:), 'b-.', 'DisplayName', 'Z Axis');
quiver3(0,0,0,x_body(1,1), x_body(2,1), x_body(3,1), 'r', 'AutoScaleFactor', 1, 'DisplayName', 'X Initial');
quiver3(0,0,0,y_body(1,1), y_body(2,1), y_body(3,1), 'g', 'AutoScaleFactor', 1, 'DisplayName', 'Y Initial');
quiver3(0,0,0,z_body(1,1), z_body(2,1), z_body(3,1), 'b', 'AutoScaleFactor', 1, 'DisplayName', 'Z Initial');
xlabel('Inertial I'); ylabel('Inertial J'); zlabel('Inertial K');
title('Orientation of Body Axes (wrt Inertial) over Time');
view(3); legend(); axis equal;

return_flag = 1;
end
