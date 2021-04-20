% Kaitlin Dennison
% Joshua Geiser

clear all; close all; clc;

%% 3-1)
%{
    Program Kinematic equations of motion correspondent to a nominal attitude parameterization.
%}

% see propagation/int_quaternion.m

%% 3-2)
%{
    Program Kinematic equations of motion correspondent to a different attitude parameterization from 1.
    (either Euler sequence or Quaternions). This is used for comparison, to get familiar with different approach,
    and as back-up in the case of singularities with 1.
%}

% see propagation/int_Euler_Angles.m

%% 3-3)
%{
    Numerically integrate Euler AND Kinematic equations from arbitrary initial conditions (warning: stay far
    from singularity of adopted parameterization). Multiple revolutions. The output is the evolution of the
    attitude parameters over time. These attitude parameters describe orientation of principal axes relative to
    inertial axes.
%}

visors = visorsStruct();

% Initial Conditions
w0 = deg2rad([-3;2;1]);
q0 = [0; 0; 0; 1];

% Sim time parameters
t0 = 0; dt = 0.5; tf = visors.T*2; t_arr = (t0:dt:tf)';

% {
% Propagate angular velocity and attitude
[w_body, quat_out] = propagate_attitude(t_arr, w0, q0);

figure(); hold on; grid on;
plot(t_arr'./60, w_body(1,:), 'DisplayName', '\omega_x');
plot(t_arr'./60, w_body(2,:), 'DisplayName', '\omega_y');
plot(t_arr'./60, w_body(3,:), 'DisplayName', '\omega_z');
xlabel('t (min)'); ylabel('\omega (rad/s)');
legend(); axis([0 12 -0.1 0.1]);
hold off

figure(); hold on; grid on;
plot(t_arr'./60, quat_out(1,:), 'DisplayName', 'q_1');
plot(t_arr'./60, quat_out(2,:), 'DisplayName', 'q_2');
plot(t_arr'./60, quat_out(3,:), 'DisplayName', 'q_3');
plot(t_arr'./60, quat_out(4,:), 'DisplayName', 'q_4');
xlabel('t (min)'); ylabel('q'); 
legend(); axis([0 12 -1 1]);
hold off
%}

%% 3-4)
%{
Since inertial position, velocity, and attitude, are known at the same time throughout the simulation, it is
now possible to express vectors in the reference systems of interest
    a. Compute angular momentum vector in inertial coordinates and verify that it is constant (not only
    its magnitude as in PS2) by plotting its components.
    b. Compute angular velocity vector in inertial coordinates and plot the herpolhode in 3D (line drawn
    in inertial space by angular velocity). Is the herpolhode contained in a plane perpendicular to the
    angular momentum vector? Show it.
    c. Compute and plots unit vectors of orbital frame, body axes, and principal axes in 3D as a function
    of time in inertial coordinates. (Be creative on how to show moving vectors in 3D).
%}
%{
% Calculations for inertial angular momentum/velocity
L_body = visors.I_princ * w_body;
L_inert = zeros(3,length(t_arr));
w_inert = zeros(3,length(t_arr));
for i = 1:length(t_arr)
    DCM = quat2dcm(quat_out(:,i));
    L_inert(:,i) = DCM' * L_body(:,i);
    w_inert(:,i) = DCM' * w_body(:,i);
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
%}

%% 3-4b)
%{
figure(); hold on; grid on; axis equal;
plot3(w_inert(1,:), w_inert(2,:), w_inert(3,:));
quiver3(0,0,0,L_inert(1,1),L_inert(2,1),L_inert(3,1), 10);
xlabel('Inertial I'); ylabel('Inertial J'); zlabel('Inertial K');
title('Herpolhode and Angular Momentum Vector in 3D Inertial Space');
legend('Herpolhode', 'Angular Momentum');
view(3);
%}

%% 3-4c)
% Get ECI2RTN for all timesteps
opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
[~,rv_array] = ode113(@FODEint,t_arr,[visors.r_ECI_0;visors.v_ECI_0],opts);
N = numel(t_arr);
DCM_ECI2RTN = zeros(3,3,N);
for n = 1:N
    DCM_ECI2RTN(:,:,n) = rotECItoRTN(visors.Om,visors.incl,visors.w,...
        visors.e,rv_array(n,1:3)');
end

% {
figure(); hold on; grid on;
triad_inert = 30.*eye(3);

% Only iterate over a subset of the simulated times
i_indices = 1:2:50;

% Transparency values for plotting triads over time
transp_i = linspace(1, 0, i_indices(end));

for i = i_indices

    DCM = quat2dcm(quat_out(:,i));
    triad_prin = DCM * triad_inert;
    triad_body = visors.A_rot' * triad_prin;
    triad_RTN = DCM_ECI2RTN(:,:,i) * triad_inert;

    % principal axes
    quiver3(0,0,0,triad_prin(1,1),triad_prin(2,1),triad_prin(3,1),...
        'Color', [1 transp_i(i) transp_i(i)],'LineWidth',2,'DisplayName','X-Principle');
    quiver3(0,0,0,triad_prin(1,2),triad_prin(2,2),triad_prin(3,2),...
        'Color', [transp_i(i) 1 transp_i(i)],'LineWidth',2,'DisplayName','Y-Principle');
    quiver3(0,0,0,triad_prin(1,3),triad_prin(2,3),triad_prin(3,3),...
        'Color', [transp_i(i) transp_i(i) 1],'LineWidth',2,'DisplayName','Z-Principle');

    % body axes
    quiver3(0,0,0,triad_body(1,1),triad_body(2,1),triad_body(3,1),...
        'Color', [1 transp_i(i) transp_i(i)], 'LineStyle', '--', 'LineWidth',2,'DisplayName','X-Body')
    quiver3(0,0,0,triad_body(1,2),triad_body(2,2),triad_body(3,2),...
        'Color', [transp_i(i) 1 transp_i(i)], 'LineStyle', '--', 'LineWidth',2,'DisplayName','Y-Body')
    quiver3(0,0,0,triad_body(1,3),triad_body(2,3),triad_body(3,3),...
        'Color', [transp_i(i) transp_i(i) 1], 'LineStyle', '--', 'LineWidth',2,'DisplayName','Z-Body')
    
    % RTN axes
    quiver3(0,0,0,triad_RTN(1,1),triad_RTN(2,1),triad_RTN(3,1),...
        'Color', [1 transp_i(i) transp_i(i)], 'LineStyle', ':', 'LineWidth',2,'DisplayName','R')
    quiver3(0,0,0,triad_RTN(1,2),triad_RTN(2,2),triad_RTN(3,2),...
        'Color', [transp_i(i) 1 transp_i(i)], 'LineStyle', ':', 'LineWidth',2,'DisplayName','T')
    quiver3(0,0,0,triad_RTN(1,3),triad_RTN(2,3),triad_RTN(3,3),...
        'Color', [transp_i(i) transp_i(i) 1], 'LineStyle', ':', 'LineWidth',2,'DisplayName','N')

end
xlabel('Inertial I'); ylabel('Inertial J'); zlabel('Inertial K');
% title('Principal Axes vs Time');
view(3);

%}

%% 3-5)
%{
Equilibrium tests
    a. Assume that 2 components of the initial angular velocities are zero, and that the principal axes are
    aligned with the inertial frame (e.g., zero Euler angles). Verify that during the simulation the 2
    components of angular velocity remain zero, and that the attitude represents a pure rotation about
    the rotation axis (e.g., linearly increasing Euler angle). Plot velocities and angles.

    b. Repeat a. by setting the initial attitude to match the RTN frame. Set the initial angular velocity to
    be non-zero only about N. Show the evolution of attitude motion in the RTN frame and give an
    interpretation of the results (recall that you might have J2 effects in orbit propagation, consider
    removing them for verification).
%}

w0_z = [0;0;w0(3)]; % rotation only about inertial z
qns_eq = [0;0;0;1]; % = dcm2quat(eye(3)); % quaterions for all eul ang = 0

t_arr_min = t_arr./60;

% {
[w_out_z, qns_out_z] = propagate_attitude(t_arr, w0_z, qns_eq);

    % Angular Velocity
    figure(); hold on; grid on;

        plot(t_arr_min,w_out_z(1,:),'DisplayName','\omega_x ','LineWidth',2,'LineStyle',':')
        plot(t_arr_min,w_out_z(2,:),'DisplayName','\omega_y ')
        plot(t_arr_min,w_out_z(3,:),'DisplayName','\omega_z ')
        xlabel('t (min)')
        ylabel('\omega (rad/s)')
        axis([0,12,-0.03,0.03])
        legend()

    hold off
    
    % Quaternions
    figure(); hold on; grid on;

        plot(t_arr_min,qns_out_z(1,:),'DisplayName','q_1','LineWidth',2,'LineStyle',':')
        plot(t_arr_min,qns_out_z(2,:),'DisplayName','q_2')
        plot(t_arr_min,qns_out_z(3,:),'DisplayName','q_3')
        plot(t_arr_min,qns_out_z(4,:),'DisplayName','q_4')
        xlabel('t (min)')
        ylabel('q')
        axis([0,12,-1,1])
        legend()

    hold off

    % Euler Angles
    A = quat2dcm(qns_out_z);
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
%}

w0_eq2 = DCM_ECI2RTN(:,:,1)'*[0;0;w0(3)]; % rotation only about N
qns_eq2 = dcm2quat(DCM_ECI2RTN(:,:,1)); % quaterions for alignment to RTN

% {
[w_out_eq2, qns_out_eq2] = propagate_attitude(t_arr, w0_eq2, qns_eq2);
w_out_eq2_rtn = zeros(size(w_out_eq2));
for n = 1:N
    w_out_eq2_rtn(:,n) = DCM_ECI2RTN(:,:,n)*w_out_eq2(:,n);
end

    % Angular Velocity
    w_max = 2*max(max(abs(w_out_eq2)));
    figure(); hold on; grid on;

        plot(t_arr_min,w_out_eq2(1,:),'DisplayName','\omega_x ECI','Color',[0, 0.4470, 0.7410])
        plot(t_arr_min,w_out_eq2_rtn(1,:),'DisplayName','\omega_x RTN','LineStyle',':','Color',[0, 0.4470, 0.7410])
        plot(t_arr_min,w_out_eq2(2,:),'DisplayName','\omega_y ECI','Color',[0.8500, 0.3250, 0.0980])
        plot(t_arr_min,w_out_eq2_rtn(2,:),'DisplayName','\omega_y RTN','LineStyle',':','Color',[0.8500, 0.3250, 0.0980])
        plot(t_arr_min,w_out_eq2(3,:),'DisplayName','\omega_z ECI','Color',[0.9290, 0.6940, 0.1250])
        plot(t_arr_min,w_out_eq2_rtn(3,:),'DisplayName','\omega_z RTN','LineStyle',':','Color',[0.9290, 0.6940, 0.1250])
        xlabel('t (min)')
        ylabel('\omega_x (rad/s)')
        axis([0,30,-w_max,w_max])
        legend('Location','southeast')

    hold off
    
    % Quaternions
    figure(); hold on; grid on;

        plot(t_arr_min,qns_out_eq2(1,:),'DisplayName','q_1')
        plot(t_arr_min,qns_out_eq2(2,:),'DisplayName','q_2')
        plot(t_arr_min,qns_out_eq2(3,:),'DisplayName','q_3')
        plot(t_arr_min,qns_out_eq2(4,:),'DisplayName','q_4')
        xlabel('t (min)')
        ylabel('q')
        legend('Location','southeast')
        axis([0,30,-1,1])

    hold off

    % Euler Angles
    A = quat2dcm(qns_out_eq2);
    eulax = dcm2eulax(A);
    figure(); hold on

        subplot(2,1,1); hold on; grid on;
        plot(t_arr_min,eulax(1,:),'DisplayName','e_x')
        plot(t_arr_min,eulax(2,:),'DisplayName','e_y')
        plot(t_arr_min,eulax(3,:),'DisplayName','e_z')
        xlabel('t (min)')
        ylabel('e')
        axis([0,30,-1,1])
        legend('Location','southeast')
        hold off

        subplot(2,1,2); hold on; grid on;
        plot(t_arr_min,eulax(4,:))
        xlabel('t (min)')
        ylabel('\Phi (rad)')
        axis([0,30,0,pi])
        hold off

    hold off
%}


%% 3-6)
%{
Stability tests
    a. Pretend you have a single-spin satellite. Set initial conditions to correspond alternatively to the 3
    possible equilibrium configurations (rotation about principal axes of inertia). Slightly perturb initial
    condition. Is the attitude stable or unstable? In angles and velocities? If stable, periodically or
    asymptotically? Show it.
%}

w0_x = [w0(3);0;0]; % rotation only about inertial x
w0_y = [0;w0(3);0]; % rotation only about inertial y
w0_z = [0;0;w0(3)]; % rotation only about inertial z
qns_eq = [0;0;0;1]; % = dcm2quat(eye(3)); % quaterions for all eul ang = 0

w_pert = -w0(3).*[.01;.01;.01];

% {
[w_out_x, qns_out_x] = propagate_attitude(t_arr, w0_x+w_pert, qns_eq);
[w_out_y, qns_out_y] = propagate_attitude(t_arr, w0_y+w_pert, qns_eq);
[w_out_z, qns_out_z] = propagate_attitude(t_arr, w0_z+w_pert, qns_eq);

    %% Angular Velocity
    w_max = 2*max(max(abs(w_out_z)));
%     figure(); hold on
    
        % w_x

        figure(); hold on; grid on;
        plot(t_arr_min,w_out_x(1,:),'DisplayName','\omega_x')
        plot(t_arr_min,w_out_x(2,:),'DisplayName','\omega_y','LineWidth',2,'LineStyle',':')
        plot(t_arr_min,w_out_x(3,:),'DisplayName','\omega_z')
        xlabel('t (min)')
        ylabel('\omega (rad/s)')
        axis([0,12*4,-0.03,0.03])
        legend()
%         title('X-Spin')
        hold off
        
        figure(); hold on; grid on;
        plot(t_arr_min,w_out_y(1,:),'DisplayName','\omega_x','LineWidth',2,'LineStyle',':')
        plot(t_arr_min,w_out_y(2,:),'DisplayName','\omega_y')
        plot(t_arr_min,w_out_y(3,:),'DisplayName','\omega_z')
        xlabel('t (min)')
        ylabel('\omega (rad/s)')
        axis([0,12*4,-0.03,0.03])
        legend()
%         title('Y-Spin')
        hold off
        
        figure(); hold on; grid on;
        plot(t_arr_min,w_out_z(1,:),'DisplayName','\omega_x','LineWidth',2,'LineStyle',':')
        plot(t_arr_min,w_out_z(2,:),'DisplayName','\omega_y')
        plot(t_arr_min,w_out_z(3,:),'DisplayName','\omega_z')
        xlabel('t (min)')
        ylabel('\omega (rad/s)')
        axis([0,12*4,-0.03,0.03])
        legend()
%         title('Z-Spin')
        hold off

    
    
    % Quaternions
%     figure(); hold on
        
        figure(); hold on; grid on;
        plot(t_arr_min,qns_out_x(1,:),'DisplayName','q_1')
        plot(t_arr_min,qns_out_x(2,:),'DisplayName','q_2','LineWidth',2,'LineStyle',':')
        plot(t_arr_min,qns_out_x(3,:),'DisplayName','q_3')
        plot(t_arr_min,qns_out_x(4,:),'DisplayName','q_4')
        xlabel('t (min)')
        ylabel('q')
%         title('X-Spin')
        legend()
        axis([0,12*4,-1,1])
        hold off
        
        figure(); hold on; grid on;
        plot(t_arr_min,qns_out_y(1,:),'DisplayName','q_1')
        plot(t_arr_min,qns_out_y(2,:),'DisplayName','q_2','LineWidth',2,'LineStyle',':')
        plot(t_arr_min,qns_out_y(3,:),'DisplayName','q_3')
        plot(t_arr_min,qns_out_y(4,:),'DisplayName','q_4')
        xlabel('t (min)')
        ylabel('q')
%         title('Y-Spin')
        legend()
        axis([0,12*4,-1,1])
        hold off
        
        figure(); hold on; grid on;
        plot(t_arr_min,qns_out_z(1,:),'DisplayName','q_1')
        plot(t_arr_min,qns_out_z(2,:),'DisplayName','q_2','LineWidth',2,'LineStyle',':')
        plot(t_arr_min,qns_out_z(3,:),'DisplayName','q_3')
        plot(t_arr_min,qns_out_z(4,:),'DisplayName','q_4')
        xlabel('t (min)')
        ylabel('q')
%         title('Z-Spin')
        legend()
        axis([0,12*4,-1,1])
        hold off

    hold off

    % Euler Angles
    A_x = quat2dcm(qns_out_x);
    eulax_x = dcm2eulax(A_x);
    A_y = quat2dcm(qns_out_y);
    eulax_y = dcm2eulax(A_y);
    A_z = quat2dcm(qns_out_z);
    eulax_z = dcm2eulax(A_z);
    
    figure(); hold on
        
        subplot(1,2,1); hold on; grid on;
        plot(t_arr_min,eulax_x(1,:),'DisplayName','e_x')
        plot(t_arr_min,eulax_x(2,:),'DisplayName','e_y')
        plot(t_arr_min,eulax_x(3,:),'DisplayName','e_z','LineWidth',2,'LineStyle',':')
        xlabel('t (min)')
        ylabel('e')
        axis([0,12*4,-1,1])
        legend()
        hold off
        
        subplot(1,2,2); hold on; grid on;
        plot(t_arr_min,eulax_x(4,:))
        xlabel('t (min)')
        ylabel('\Phi (rad)')
        axis([0,12*4,-pi,pi])
        hold off
        
    hold off
    figure(); hold on
        
        subplot(1,2,1); hold on; grid on;
        plot(t_arr_min,eulax_y(1,:),'DisplayName','e_x','LineWidth',2,'LineStyle',':')
        plot(t_arr_min,eulax_y(2,:),'DisplayName','e_y')
        plot(t_arr_min,eulax_y(3,:),'DisplayName','e_z')
        xlabel('t (min)')
        ylabel('e')
        axis([0,12*4,-1,1])
        legend()
        hold off
        
        subplot(1,2,2); hold on; grid on;
        plot(t_arr_min,eulax_y(4,:))
        xlabel('t (min)')
        ylabel('\Phi (rad)')
        axis([0,12*4,-pi,pi])
        hold off
        
    hold off
    figure(); hold on
        
        subplot(1,2,1); hold on; grid on;
        plot(t_arr_min,eulax_z(1,:),'DisplayName','e_x','LineWidth',2,'LineStyle',':')
        plot(t_arr_min,eulax_z(2,:),'DisplayName','e_y')
        plot(t_arr_min,eulax_z(3,:),'DisplayName','e_z')
        xlabel('t (min)')
        ylabel('e')
        axis([0,12*4,-1,1])
        legend()
        hold off
        
        subplot(1,2,2); hold on; grid on;
        plot(t_arr_min,eulax_z(4,:))
        xlabel('t (min)')
        ylabel('\Phi (rad)')
        axis([0,12*4,-pi,pi])
        hold off

    hold off
