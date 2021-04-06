% Kaitlin Dennison
% Joshua Geiser

clear;clc;close all;

visorsModel;
% f_body, v_body, f_panel, v_panel_1, v_panel_2

%% 1-6) 
%{
    Develop a Matlab/Simulink function to handle barycenter coordinates, 
    size, and unit vectors normal to each outer surface of the spacecraft
    in the body frame.
%}

n_body = size(f_body,1);
n_panel = size(f_panel,1);

% main body
c_body = zeros(n_body,3);
a_body = zeros(n_body,1);
u_body = zeros(n_body,3);
for i = 1:n_body
    verts = v_body(f_body(i,:)',:);
    c_body(i,:) = mean(verts,1);
    a_body(i) = getArea(verts);
    u_body(i,:) = getNormal(verts);
end

% manually adjust  unit vec signs
u_body([1,3,6],:) = -u_body([1,3,6],:);

% solar panels
c_panel_1 = zeros(n_panel,3);
a_panel_1 = zeros(n_panel,1);
c_panel_2 = zeros(n_panel,3);
a_panel_2 = zeros(n_panel,1);
for i = 1:n_panel
    verts = v_panel_1(f_panel(i,:)',:);
    c_panel_1(i,:) = mean(verts,1);
    a_panel_1(i) = getArea(verts);
    verts = v_panel_2(f_panel(i,:)',:);
    c_panel_2(i,:) = mean(verts,1);
    a_panel_2(i) = getArea(verts);
end

% panels are a rotation of body
u_panel = (R3(pi/4)*u_body')';
u_panel_1 = (R3(pi)*u_panel')';


%% 1-7) Plot the body axis
%{
    Plot the body axis (triad) superimposed on the spacecraft 3D model
%}

%{
figure(); hold on
    
    % triad 
    quiver3(zeros(3,1),zeros(3,1),zeros(3,1),[30;0;0],[0;0;0],[0;0;0],'r')
    quiver3(zeros(3,1),zeros(3,1),zeros(3,1),[0;0;0],[0;30;0],[0;0;0],'g')
    quiver3(zeros(3,1),zeros(3,1),zeros(3,1),[0;0;0],[0;0;0],[0;0;30],'c')
    
    % 3D model
    patch('Faces',f_body,'Vertices',v_body,'FaceColor',[0.5,0.5,0.5])
    patch('Faces',f_panel,'Vertices',v_panel_1,'FaceColor','blue')
    patch('Faces',f_panel,'Vertices',v_panel_2,'FaceColor','blue')
    
    xlabel('X (cm)')
    ylabel('Y (cm)')
    zlabel('Z (cm)')
    view(37.5+90,30)
    axis equal
    
hold off
%}
 
%% 1-8) Define and Plot the Orbit
 
visorsOrbit

[r,v] = ConvOEtoRV(oe,mu);
opts = odeset('abstol', 1e-3, 'reltol', 1e-3);
[t_array,rv_array] = ode45(@FODEint,0:(2*60):(2*T),[r;v],opts);
% % opts = odeset('abstol', 1e-6, 'reltol', 1e-6);
% [t_array,rv_array] = ode45(@FODEint,linspace(0,2*T,100)',[r;v]);

figure(); hold on
    plot3(rv_array(:,1),rv_array(:,2),rv_array(:,3),'r')
    earth_sphere('m')
    axis equal
    hold off
    

%% Functions
 
function U = getNormal(verts)
    v1 = verts(2,:) - verts(1,:);
    v2 = verts(4,:) - verts(1,:);
    U = unitVec(cross(v1,v2))'; 
end

function A = getArea(verts)
    d1 = norm(verts(1,:)-verts(2,:));
    d2 = norm(verts(1,:)-verts(4,:));
    d3 = norm(verts(1,:)-verts(4,:));
    if d1 == d2
        A = d1 * d3;
    else
        A = d1 * d2;
    end
end