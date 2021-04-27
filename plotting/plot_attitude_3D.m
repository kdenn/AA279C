function return_flag = plot_attitude_3D(visors, quat_out)
% Provide 3D Visualizations of the attitude evolution over time. One plot
% for principal axes and one plot for body-fixed axes. 
%
% Inputs: visors: visors struct object
%         quat_out: time history of quaternion (4xN vector)

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
