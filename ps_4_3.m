% Kaitlin Dennison
% Joshua Geiser

clear all; close all; clc;

%% 3a) Gravity Gradient Torque (Stability)
%{
Calculate the coefficients Ki of the moments of inertia which drive stability under gravity gradient.
Compute and plot regions of stable and unstable motion similar to the picture below:
%}

visors = visorsStruct();
Ix = visors.Ix; Iy = visors.Iy; Iz = visors.Iz;

N = 500;

kt_1_arr = linspace(-1,1,N);
kr_1_arr = 1 .* kt_1_arr;

kt_2_arr = linspace(-1,-0.01,N);
kr_2_arr = [];
for kt_2 = kt_2_arr
    syms kr_2
    eqn = (1 + 3.*kt_2 + kr_2.*kt_2).^2 - 16.*kr_2.*kt_2 == 0;
    S = vpasolve(eqn, [kr_2]);
    kr_2_arr = [kr_2_arr double(S(2))];
end

%%
figure(); hold on; grid on;

visors_kt = (Iz-Ix)/Iy;
visors_kr = (Iz-Iy)/Ix;
visors_kn = (Iy-Ix)/Iz;
plot(visors_kt, visors_kr, 'r*', 'DisplayName', 'Visors');
xlabel('K_T'); ylabel('K_R');
axis([-1 1 -1 1]);

patch([0,1,1,0],[0,0,-1,-1], 'b', 'FaceAlpha', 0.75);
patch([0,1,0],[0,1,1], 'y', 'FaceAlpha', 0.75);
patch([-1,0,0,-1],[0,0,1,1], 'g', 'FaceAlpha', 0.75);
legend({'Visors', 'Unstable Yaw and Roll', 'Unstable Pitch', 'Unstable Yaw, Roll, and Pitch'}, ...
        'AutoUpdate', 'off', 'Location', 'Northwest');
plot(kt_1_arr, kr_1_arr, 'k');
plot(kt_2_arr, kr_2_arr, 'k');
xline(0); yline(0);

xc = -0.145; yc = -0.145;
patch([kt_1_arr(kt_1_arr<xc) fliplr(kt_2_arr(kt_2_arr<xc))], ...
    [kr_1_arr(kt_1_arr<xc) fliplr(kr_2_arr(kt_2_arr<xc))], ...
    'g', 'FaceAlpha', 0.75);
patch([kt_1_arr(kt_1_arr<xc) kt_2_arr(kt_2_arr>xc)], ...
    [kr_1_arr(kt_1_arr<xc) kr_2_arr(kt_2_arr>xc)], ...
    'b', 'FaceAlpha', 0.75);
patch([kt_2_arr(kt_2_arr<(-1/3)) (-1/3) -1], ...
    [kr_2_arr(kt_2_arr<(-1/3)) 0 0], ...
    'y', 'FaceAlpha', 0.75);
patch([kt_2_arr((-1/3)<kt_2_arr & kt_2_arr<xc) kt_1_arr(xc<kt_1_arr & kt_1_arr<0) 0], ...
    [kr_2_arr((-1/3)<kt_2_arr & kt_2_arr<xc) kr_1_arr(xc<kt_1_arr & kt_1_arr<0) 0], ...
    'y', 'FaceAlpha', 0.75);

%% 4-3-c
visors = visorsStruct();
colors = DefaultColors();

% Sim time parameters
t0 = 0; dt = 1; tf = visors.T*4; t_arr = (t0:dt:tf)';

% Unstable Sun-pointing
w0_sun = visors.A_rot*[visors.n;0;0]; % spin about x-body
q0_sun = dcm2quat(visors.A_rot); % body/ECI -> princ

% Stable??
w0_stable = [visors.n;0;0];
q0_stable = [0;0;0;1];

w_pert = 0.01*visors.n*ones(3,1); % 1 percent of the value

%%
% {
% Propagate angular velocity and attitude
[w_body_s, quat_out_s, rv_ECI_out_s, M_out_s] = propagate_attitude(t_arr, w0_stable+w_pert, q0_stable, 1);
[w_body, quat_out, rv_ECI_out, M_out] = propagate_attitude(t_arr, w0_sun+w_pert, q0_sun, 1);

%%
figure(); hold on; grid on;
plot(t_arr'./60, quat_out(1,:), 'DisplayName', 'q_1 sun','LineStyle','-','Color',colors(1,:));
plot(t_arr'./60, quat_out(2,:), 'DisplayName', 'q_2 sun','LineStyle','-','Color',colors(2,:));
plot(t_arr'./60, quat_out(3,:), 'DisplayName', 'q_3 sun','LineStyle','-','Color',colors(3,:));
plot(t_arr'./60, quat_out(4,:), 'DisplayName', 'q_4 sun','LineStyle','-','Color',colors(4,:));
xlabel('t (min)'); ylabel('q'); 
legend(); 
set(gcf, 'Position',  [100, 100, 800, 300]);
hold off

figure(); hold on; grid on;
plot(t_arr'./60, quat_out_s(1,:), 'DisplayName', 'q_1 stable','LineStyle','-','Color',colors(1,:));
plot(t_arr'./60, quat_out_s(2,:), 'DisplayName', 'q_2 stable','LineStyle','-','Color',colors(2,:));
plot(t_arr'./60, quat_out_s(3,:), 'DisplayName', 'q_3 stable','LineStyle','-','Color',colors(3,:));
plot(t_arr'./60, quat_out_s(4,:), 'DisplayName', 'q_4 stable','LineStyle','-','Color',colors(4,:));
xlabel('t (min)'); ylabel('q'); 
legend(); 
set(gcf, 'Position',  [100, 100, 800, 300]);
hold off

%}