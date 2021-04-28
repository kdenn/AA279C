% Faces and Vertices of the Visors model
R_body = R2(-pi/2)*R3(-pi/2);

%% 6U Bus
d = 10; % side length (cm)

f_bus = [1 2 3 4;
         3 4 5 6;
         5 6 7 8;
         7 8 1 2;
         2 3 6 7;
         1 4 5 8];
      
v_bus = [ 1 -0.5 -1.5;
          1  0.5 -1.5;
         -1  0.5 -1.5;
         -1 -0.5 -1.5;
         -1 -0.5  1.5;
         -1  0.5  1.5;
          1  0.5  1.5;
          1 -0.5  1.5].*d;

v_bus = (R_body*v_bus')';

%% Solar Panels
s = 20; % width of solar panel section (cm)
t = 0.2; % solar panel thickness

f_pan = [1 2 3 4;
         3 4 5 6;
         5 6 7 8;
         7 8 1 2;
         2 3 6 7;
         1 4 5 8];

v_base = [s*3 -0.5*t -1.5*d;
          s*3  0.5*t -1.5*d;
          0    0.5*t -1.5*d;
          0   -0.5*t -1.5*d;
          0   -0.5*t  1.5*d;
          0    0.5*t  1.5*d;
          s*3  0.5*t  1.5*d;
          s*3 -0.5*t  1.5*d];
       
v_45 = (R2(pi/2)*v_base')';
body_offset = repmat([0 0 1.5*d],[8,1]);
v_pan_1 = v_45 + body_offset;
v_pan_2 = (R2(pi)*v_pan_1')';

v_pan_1 = (R_body*v_pan_1')';
v_pan_2 = (R_body*v_pan_2')';

v_pan_cen = v_base - repmat([s*1.5 0 0],[8,1]);

%% Plot
%{
patch('Faces',f_body,'Vertices',v_body,'FaceColor','red')
hold on
axis equal
patch('Faces',f_pan,'Vertices',v_pan,'FaceColor','blue')
patch('Faces',f_pan,'Vertices',v_pan_2,'FaceColor','blue')
view(3)
%}