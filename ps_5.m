% Kaitlin Dennison
% Joshua Geiser

clear all; close all; clc;
colors = DefaultColors();
repropagate = true;

%% 5-1)
%{
    In addition to gravity gradient, program perturbation torques due to
    magnetic field, solar radiation pressure, and  atmospheric  drag. Note
    1: You  can  apply  a  very  minimal/basic  model  for  perturbations
    that are  not  relevant (negligible) to your project. Note 2: All
    perturbations can be grouped into a single large subsystem called
    environment or similar whose output feed the Euler equations. Note 3:
    Re-use as many functions as possible for solar radiation pressure and
    atmospheric drag.
%}
if ~exist('data/5-1.mat','file') || repropagate
    % Initial Conditions
    w0 = deg2rad([0;0.1;1]);
    q0 = [0; 0; 0; 1];

    % Sim time parameters
    t0 = 0; dt = 0.5; tf = 60*10; t_arr = (t0:dt:tf)';

    % Visors class (not struct)
    visors = Visors(w0, q0);

    [omega_out, quat_out, rv_ECI_out, M_out] = visors.propagate(t_arr,[1,1,1,1]);

    plot_attitude_3D(visors.ICs, quat_out);

    save('data/5-1.mat')
else
    load('data/5-1.mat')
end

%% 5-2)
%{
    Include all torques in numerical integration. Please show comparison of
    numerically computed disturbance torques with expected values and trend
    from theory (model) and tables (Wertz). Plot all torque components in
    principal axes over time. Plot the resultant (sum) of all torques in
    principal axes. Make sure that your model is not too ideal, i.e. make
    sure that center of pressure and center of mass do not coincide.
%}

names = {'Gravity','SRP','Drag','Magnetic','Total'};

% Please show comparison of numerically computed disturbance torques with
% expected values and trend from theory (model) and tables (Wertz).

faces = visors.ICs.faces;
As = faces.area([5,11,18],:);
rs = faces.bary([5,11,18],:);

P = 1358/(3e8);

df = -(11/9).*P.*As;
df = [zeros(3,1),df,zeros(3,1)]';
M_s = cross(rs, df, 2);
M_s = norm(sum(M_s,2))

Cd = 1.28;
rho = 1.454e-13;

v = sqrt(visors.ICs.mu/visors.ICs.a);

df = -0.5*Cd*rho*v^2.*As;
df = [zeros(3,1),df,zeros(3,1)]';
M_d = cross(rs, df, 2);
M_d = norm(sum(M_s,2))


% Plot all torque components in principal axes over time. 
figure(); hold on; 

    subplot(3,1,1); hold on; grid on;
        ylabel('M_x')
        for i = 1:5
            plot(t_arr'./60, M_out(1,:,i), 'DisplayName', names{i},'Color',colors(i,:));
        end
        legend();
    hold off

    subplot(3,1,2); hold on; grid on;
        ylabel('M_y')
        for i = 1:5
            plot(t_arr'./60, M_out(2,:,i), 'DisplayName', names{i},'Color',colors(i,:));
        end
    hold off

    subplot(3,1,3); hold on; grid on;
        xlabel('t (min)')
        ylabel('M_z')
        for i = 1:5
            plot(t_arr'./60, M_out(3,:,i), 'DisplayName', names{i},'Color',colors(i,:));
        end
    hold off

set(gcf, 'Position',  [100, 100, 800, 700]);
hold off

figure(); hold on; 

    subplot(3,1,1); hold on; grid on;
        ylabel('M_x')
        for i = 4
            plot(t_arr'./60, M_out(1,:,i), 'DisplayName', names{i},'Color',colors(i,:));
        end
        legend();
    hold off

    subplot(3,1,2); hold on; grid on;
        ylabel('M_y')
        for i = 4
            plot(t_arr'./60, M_out(2,:,i), 'DisplayName', names{i},'Color',colors(i,:));
        end
    hold off

    subplot(3,1,3); hold on; grid on;
        xlabel('t (min)')
        ylabel('M_z')
        for i = 4
            plot(t_arr'./60, M_out(3,:,i), 'DisplayName', names{i},'Color',colors(i,:));
        end
    hold off

set(gcf, 'Position',  [100, 100, 800, 700]);
hold off

figure(); hold on; 

    subplot(3,1,1); hold on; grid on;
        ylabel('M_x')
        for i = 2:3
            plot(t_arr'./60, M_out(1,:,i), 'DisplayName', names{i},'Color',colors(i,:));
        end
        legend();
    hold off

    subplot(3,1,2); hold on; grid on;
        ylabel('M_y')
        for i = 2:3
            plot(t_arr'./60, M_out(2,:,i), 'DisplayName', names{i},'Color',colors(i,:));
        end
    hold off

    subplot(3,1,3); hold on; grid on;
        xlabel('t (min)')
        ylabel('M_z')
        for i = 2:3
            plot(t_arr'./60, M_out(3,:,i), 'DisplayName', names{i},'Color',colors(i,:));
        end
    hold off

set(gcf, 'Position',  [100, 100, 800, 700]);
hold off

return

%% 5-3 and 5-4)
%{
    Now you can compute the attitude control error even if a controller is
    not implemented yet. The attitude control error represents the rotation
    between the desired and actual attitude. Plot the attitude control
    error and  give its  interpretation.  Note  that  this  step  requires
    the  definition and  computation  of  the  desired  or  nominal or
    target attitude of the spacecraft. In general, this can be expressed in
    body or principal axes.

    Note that the attitude control error represents a rotation matrix (DCM)
    which quantifies how far the actualattitude  is  from  the  true
    attitude. You  can  use  any  parameterization to  plot  the  attitude
    control errors corresponding to this DCM. Give interpretation of the
    attitude control errors given the applied disturbances.
%}

% Initial Conditions
s = visorsStruct();
w0 = deg2rad([0;0;0]);
q0 = [-0.24955978816328; 0.661604044827341; 0.661761547188027; 0.249141836437215];

% Sim time parameters
t0 = 0; dt = 0.5; tf = s.T; t_arr = (t0:dt:tf)';

% Visors class (not struct)
visors = Visors(w0, q0);

[omega_out, quat_out, rv_ECI_out, M_out] = visors.propagate(t_arr);

q_des = visors.calc_q_des(t_arr);

%%
plot_attitude_3D(visors.ICs, quat_out);

%%
q_diff = quat_out - q_des;

figure(); hold on; grid on;
plot(t_arr./60, q_diff(1,:));
plot(t_arr./60, q_diff(2,:));
plot(t_arr./60, q_diff(3,:));
plot(t_arr./60, q_diff(4,:));
xlabel('Time (min)'); ylabel('Quaternion');
legend('q_1', 'q_2', 'q_3', 'q_4');

%%
angle_err = calc_angle_err(visors.ICs, quat_out, q_des);

figure(); hold on; grid on;
plot(t_arr./60, angle_err); 
xlabel('Time (min)'); ylabel('\theta_{error} (deg)'); 

function angle_err = calc_angle_err(vsr, q_true, q_des)
% Calculate the angle between actual X body axis and desired X body axis
% (desired should basically be Sun direction) given true and desired
% quaternions

N = length(q_true);
angle_err = zeros(N,1);

for i = 1:N
    
    % X direction of body axis
    DCM_true = vsr.A_rot' * quat2dcm(q_true(:,i));
    x_hat_true = DCM_true(:,1);
    
    % Desired X direction of body axis (i.e. sun direction)
    DCM_des = vsr.A_rot' * quat2dcm(q_des(:,i));
    x_hat_des = DCM_des(:,1);
    
    % Angle between the two vectors
    dot_prod = dot(x_hat_true, x_hat_des);
    angle = rad2deg(real(acos(dot_prod)));
    angle_err(i) = angle;
end
end
