% Kaitlin Dennison
% Joshua Geiser

clear;clc;close all;

%% 1-6) 
%{
    Develop a Matlab/Simulink function to handle barycenter coordinates, 
    size, and unit vectors normal to each outer surface of the spacecraft
    in the body frame.
%}

visorsModel;
% f_bus, v_bus, f_pan, v_pan_1, v_pan_2

[com_bus,c_bus,a_bus,u_bus] = spacecraftGeom(f_bus,v_bus);
[com_pan_1,c_pan_1,a_pan_1,u_pan_1] = spacecraftGeom(f_pan,v_pan_1);
[com_pan_2,c_pan_2,a_pan_2,u_pan_2] = spacecraftGeom(f_pan,v_pan_2);

%% 1-7) Plot the body axis
%{
    Plot the body axis (triad) superimposed on the spacecraft 3D model
%}

triad_body = 30.*eye(3);

figure(); hold on
    
    % triad 
    quiver3(0,0,0,triad_body(1,1),triad_body(2,1),triad_body(3,1),'r')
    quiver3(0,0,0,triad_body(1,2),triad_body(2,2),triad_body(3,2),'g')
    quiver3(0,0,0,triad_body(1,3),triad_body(2,3),triad_body(3,3),'c')
    
    % 3D model
    patch('Faces',f_bus,'Vertices',v_bus,'FaceColor',[0.5,0.5,0.5])
    
    xlabel('X (cm)')
    ylabel('Y (cm)')
    zlabel('Z (cm)')
    view(37.5+90,30)
    axis equal
    
hold off

figure(); hold on
    
    % triad 
    quiver3(0,0,0,triad_body(1,1),triad_body(2,1),triad_body(3,1),'r')
    quiver3(0,0,0,triad_body(1,2),triad_body(2,2),triad_body(3,2),'g')
    quiver3(0,0,0,triad_body(1,3),triad_body(2,3),triad_body(3,3),'c')
    
    % 3D model
    patch('Faces',f_bus,'Vertices',v_pan_cen,'FaceColor','blue')
    
    xlabel('X (cm)')
    ylabel('Y (cm)')
    zlabel('Z (cm)')
    view(37.5+90,30)
    axis equal
    
hold off


% {
figure(); hold on
    
    % triad 
    quiver3(0,0,0,triad_body(1,1),triad_body(2,1),triad_body(3,1),'r')
    quiver3(0,0,0,triad_body(1,2),triad_body(2,2),triad_body(3,2),'g')
    quiver3(0,0,0,triad_body(1,3),triad_body(2,3),triad_body(3,3),'c')
    
    % 3D model
    patch('Faces',f_bus,'Vertices',v_bus,'FaceColor',[0.5,0.5,0.5])
    patch('Faces',f_pan,'Vertices',v_pan_1,'FaceColor','blue')
    patch('Faces',f_pan,'Vertices',v_pan_2,'FaceColor','blue')
    plot3(com_pan_1(1),com_pan_1(2),com_pan_1(3),'r*')
    plot3(com_pan_2(1),com_pan_2(2),com_pan_2(3),'g*')
    
    xlabel('X (cm)')
    ylabel('Y (cm)')
    zlabel('Z (cm)')
    view(37.5+90,30)
    axis equal
    
hold off
%}
 
%% 1-8) Define and Plot the Orbit
 
visorsOrbit

% ECI propagation
[r,v] = ConvOEtoRV(oe,mu);
opts = odeset('abstol', 1e-3, 'reltol', 1e-3);
[t_array,rv_array] = ode113(@FODEint,0:30:(2*T),[r;v],opts);

% ECEF
JD_array = t_array./86400 + JD_epoch;
r_ECEF = zeros(size(rv_array,1),3);
for i = 1:numel(JD_array)
    R_ECItoECEF = rotECItoECEF(JD2GMST(JD_array(i)));
    r_ECEF(i,1:3) = (R_ECItoECEF*rv_array(i,1:3)')';
end


figure(); hold on
subplot(1,2,1); hold on
    plot3(rv_array(:,1),rv_array(:,2),rv_array(:,3),'r')
    [X,Y,Z] = sphere;
    surf(X.*s3_constants('R_EARTH'),Y.*s3_constants('R_EARTH'),Z.*s3_constants('R_EARTH'),'FaceColor','blue')
    xlabel('ECI X (m)')
    ylabel('ECI Y (m)')
    zlabel('ECI Z (m)')
    view(37.5+90,30)
    axis equal
    hold off
subplot(1,2,2); hold on
    plot3(r_ECEF(:,1),r_ECEF(:,2),r_ECEF(:,3),'r')
    earth_sphere('m')
    xlabel('ECEF X (m)')
    ylabel('ECEF Y (m)')
    zlabel('ECEF Z (m)')
    axis equal
    hold off
hold off
    