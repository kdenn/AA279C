% Kaitlin Dennison
% Joshua Geiser

clear all; close all; clc;
colors = DefaultColors();

visors = visorsStruct();
% Sim time parameters
t0 = 0; dt = 0.5; tf = visors.T*2; t_arr = (t0:dt:tf)';

%% 4-2-b)
%{
    Program gravity gradient torque. Feed torque to Euler equations. This
    is the first perturbation you model resulting from the interaction of
    the spacecraft with the environment. Hint: change your orbit to make
    gravity gradient significant if that’s not the case.
%}

% see propagate_attitude, int_Euler_eqs, grav_grad


%% 4-2-c)
%{
    Verify that the magnitude of the modelled torque is consistent with the
    orbit and inertia tensor of your satellite. Hint: use simplified
    formulas from class on modelling of gravity gradient torque.
%}

M = ((3*visors.mu)/(visors.a^3)) * [(visors.Iz - visors.Iy);
                                    (visors.Ix - visors.Iz);
                                    (visors.Iy - visors.Ix)];
M_max = max(abs(M))


%% 4-2-d)
%{
    Numerically integrate Euler and Kinematic equations including gravity
    gradient from initial conditions corresponding to body axes aligned
    with the orbital frame (RTN). Verify that gravity gradient torque is
    zero, besides numerical errors. Hint: you may need to simplify the
    orbit to unperturbed circular to achieve this. Check that initial
    angular velocity matches mean motion.
%}

% Initial Conditions
rv0 = [visors.r_ECI_0;visors.v_ECI_0];
R_ECI2RTN = rotECItoRTN(visors.Om,visors.incl,visors.w,visors.e,rv0(1:3));
w0 = [0;0;visors.n]; % princ
q0 = dcm2quat(R_ECI2RTN); % ECI -> princ

% Propagate angular velocity and attitude
% {
[w_body, quat_out, rv_ECI_out, M_out] = propagate_attitude(t_arr, w0, q0, 0);
[w_body_g, quat_out_g, rv_ECI_out_g, M_out_g] = propagate_attitude(t_arr, w0, q0, 1);

%% Verify RTN isn't changing
%{
q_RTN = zeros(size(quat_out));
for i = 1:numel(t_arr)
    R_ECI2princ = quat2dcm(quat_out(:,i));
    R_RTN2princ = R_ECI2princ*R_ECI2RTN';
    q_RTN(:,i) = dcm2quat(R_RTN2princ);
end

figure(); hold on; grid on;
plot(t_arr'./60, q_RTN(1,:), 'DisplayName', 'q_1');
plot(t_arr'./60, q_RTN(2,:), 'DisplayName', 'q_2');
plot(t_arr'./60, q_RTN(3,:), 'DisplayName', 'q_3');
plot(t_arr'./60, q_RTN(4,:), 'DisplayName', 'q_4');
xlabel('t (min)'); ylabel('q'); 
legend(); 
set(gcf, 'Position',  [100, 100, 600, 200]);
hold off
%}

%{
figure(); hold on; grid on;
plot(t_arr'./60, w_body(1,:), 'DisplayName', '\omega_x no grav','LineStyle','-','Color',colors(1,:));
plot(t_arr'./60, w_body(2,:), 'DisplayName', '\omega_y no grav','LineStyle','-','Color',colors(2,:));
plot(t_arr'./60, w_body(3,:), 'DisplayName', '\omega_z no grav','LineStyle','-','Color',colors(3,:));
plot(t_arr'./60, w_body_g(1,:), 'DisplayName', '\omega_x','LineStyle','--','Color',colors(1,:));
plot(t_arr'./60, w_body_g(2,:), 'DisplayName', '\omega_y','LineStyle','--','Color',colors(2,:));
plot(t_arr'./60, w_body_g(3,:), 'DisplayName', '\omega_z','LineStyle','--','Color',colors(3,:));
xlabel('t (min)'); ylabel('\omega (rad/s)');
legend();
set(gcf, 'Position',  [100, 100, 600, 200]);
hold off

figure(); hold on; grid on;
plot(t_arr'./60, quat_out(1,:), 'DisplayName', 'q_1 no grav','LineStyle','-','Color',colors(1,:));
plot(t_arr'./60, quat_out(2,:), 'DisplayName', 'q_2 no grav','LineStyle','-','Color',colors(2,:));
plot(t_arr'./60, quat_out(3,:), 'DisplayName', 'q_3 no grav','LineStyle','-','Color',colors(3,:));
plot(t_arr'./60, quat_out(4,:), 'DisplayName', 'q_4 no grav','LineStyle','-','Color',colors(4,:));
plot(t_arr'./60, quat_out_g(1,:), 'DisplayName', 'q_1','LineStyle','--','Color',colors(1,:));
plot(t_arr'./60, quat_out_g(2,:), 'DisplayName', 'q_2','LineStyle','--','Color',colors(2,:));
plot(t_arr'./60, quat_out_g(3,:), 'DisplayName', 'q_3','LineStyle','--','Color',colors(3,:));
plot(t_arr'./60, quat_out_g(4,:), 'DisplayName', 'q_4','LineStyle','--','Color',colors(4,:));
xlabel('t (min)'); ylabel('q'); 
legend(); 
set(gcf, 'Position',  [100, 100, 600, 200]);
hold off

figure(); hold on; grid on;
plot(t_arr'./60, M_out_g(1,:), 'DisplayName', 'M_x');
plot(t_arr'./60, M_out_g(2,:), 'DisplayName', 'M_y');
plot(t_arr'./60, M_out_g(3,:), 'DisplayName', 'M_z');
legend();
set(gcf, 'Position',  [100, 100, 600, 200]);
hold off

figure(); hold on; grid on;
plot(t_arr'./60, w_body(1,:)-w_body_g(1,:), 'DisplayName', '\delta \omega_x');
plot(t_arr'./60, w_body(2,:)-w_body_g(2,:), 'DisplayName', '\delta \omega_y');
plot(t_arr'./60, w_body(3,:)-w_body_g(3,:), 'DisplayName', '\delta \omega_z');
xlabel('t (min)'); ylabel('\delta \omega (rad/s)');
legend();
set(gcf, 'Position',  [100, 100, 600, 200]);
hold off

figure(); hold on; grid on;
plot(t_arr'./60, quat_out(1,:)-quat_out_g(1,:), 'DisplayName', '\delta q_1');
plot(t_arr'./60, quat_out(2,:)-quat_out_g(2,:), 'DisplayName', '\delta q_2');
plot(t_arr'./60, quat_out(3,:)-quat_out_g(3,:), 'DisplayName', '\delta q_3');
plot(t_arr'./60, quat_out(4,:)-quat_out_g(4,:), 'DisplayName', '\delta q_4');
xlabel('t (min)'); ylabel('\delta q'); 
legend(); 
set(gcf, 'Position',  [100, 100, 600, 200]);
hold off

return
%}

%% 4-2-e)
%{
    Numerically integrate Euler and Kinematic equations including gravity
    gradient from arbitrary initial conditions (e.g., relevant to your
    project). Plot external torque (3 components w.r.t. time) and resulting
    attitude motion (depends on attitude parameterization, add Euler angles
    for better geometrical interpretation) over multiple orbits. Comment on
    results.
%}

% Get Sun-Pointing Vector for x-body
% [r_earth_HCI_0, v_earth_HCI_0, oe_earth] = planetHCI(3, visors.JD_epoch);
% r_sun_ECI_0 = -r_earth_HCI_0;
% r_sc2sun_ECI_0 = r_sun_ECI_0 - visors.r_ECI_0;
% x_sun_ECI = unitVec(r_sc2sun_ECI_0);

% Initial Conditions
w0 = visors.A_rot*[1;0;0].*visors.n; % spin about x-body
q0 = dcm2quat(visors.A_rot); % body/ECI -> princ

% {
% Propagate angular velocity and attitude
[w_body_g, quat_out_g, rv_ECI_out_g, M_out_g] = propagate_attitude(t_arr, w0, q0, 1);
[w_body, quat_out, rv_ECI_out, M_out] = propagate_attitude(t_arr, w0, q0, 0);

%%
figure(); hold on; grid on;
plot(t_arr'./60, w_body(1,:), 'DisplayName', '\omega_x no grav','LineStyle','-','Color',colors(1,:));
plot(t_arr'./60, w_body(2,:), 'DisplayName', '\omega_y no grav','LineStyle','-','Color',colors(2,:));
plot(t_arr'./60, w_body(3,:), 'DisplayName', '\omega_z no grav','LineStyle','-','Color',colors(3,:));
plot(t_arr'./60, w_body_g(1,:), 'DisplayName', '\omega_x','LineStyle','--','Color',colors(1,:));
plot(t_arr'./60, w_body_g(2,:), 'DisplayName', '\omega_y','LineStyle','--','Color',colors(2,:));
plot(t_arr'./60, w_body_g(3,:), 'DisplayName', '\omega_z','LineStyle','--','Color',colors(3,:));
xlabel('t (min)'); ylabel('\omega (rad/s)');
legend();
set(gcf, 'Position',  [100, 100, 600, 200]);
hold off

figure(); hold on; grid on;
plot(t_arr'./60, quat_out(1,:), 'DisplayName', 'q_1 no grav','LineStyle','-','Color',colors(1,:));
plot(t_arr'./60, quat_out(2,:), 'DisplayName', 'q_2 no grav','LineStyle','-','Color',colors(2,:));
plot(t_arr'./60, quat_out(3,:), 'DisplayName', 'q_3 no grav','LineStyle','-','Color',colors(3,:));
plot(t_arr'./60, quat_out(4,:), 'DisplayName', 'q_4 no grav','LineStyle','-','Color',colors(4,:));
plot(t_arr'./60, quat_out_g(1,:), 'DisplayName', 'q_1','LineStyle','--','Color',colors(1,:));
plot(t_arr'./60, quat_out_g(2,:), 'DisplayName', 'q_2','LineStyle','--','Color',colors(2,:));
plot(t_arr'./60, quat_out_g(3,:), 'DisplayName', 'q_3','LineStyle','--','Color',colors(3,:));
plot(t_arr'./60, quat_out_g(4,:), 'DisplayName', 'q_4','LineStyle','--','Color',colors(4,:));
xlabel('t (min)'); ylabel('q'); 
legend(); 
set(gcf, 'Position',  [100, 100, 600, 200]);
hold off

figure(); hold on; grid on;
plot(t_arr'./60, M_out_g(1,:), 'DisplayName', 'M_x');
plot(t_arr'./60, M_out_g(2,:), 'DisplayName', 'M_y');
plot(t_arr'./60, M_out_g(3,:), 'DisplayName', 'M_z');
legend();
set(gcf, 'Position',  [100, 100, 600, 200]);
hold off

figure(); hold on; grid on;
plot(t_arr'./60, w_body(1,:)-w_body_g(1,:), 'DisplayName', '\delta\omega_x');
plot(t_arr'./60, w_body(2,:)-w_body_g(2,:), 'DisplayName', '\delta\omega_y');
plot(t_arr'./60, w_body(3,:)-w_body_g(3,:), 'DisplayName', '\delta\omega_z');
xlabel('t (min)'); ylabel('\delta\omega (rad/s)');
legend();
set(gcf, 'Position',  [100, 100, 600, 200]);
hold off

figure(); hold on; grid on;
plot(t_arr'./60, quat_out(1,:)-quat_out_g(1,:), 'DisplayName', '\delta q_1');
plot(t_arr'./60, quat_out(2,:)-quat_out_g(2,:), 'DisplayName', '\delta q_2');
plot(t_arr'./60, quat_out(3,:)-quat_out_g(3,:), 'DisplayName', '\delta q_3');
plot(t_arr'./60, quat_out(4,:)-quat_out_g(4,:), 'DisplayName', '\delta q_4');
xlabel('t (min)'); ylabel('\delta q'); 
legend(); 
set(gcf, 'Position',  [100, 100, 600, 200]);
hold off
%}






























