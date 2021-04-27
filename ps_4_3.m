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