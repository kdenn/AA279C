% Faces and Vertices of the Visors model

%% 6U Body
d = 10; % side length (cm)

f_body = [1 2 3 4;
          3 4 5 6;
          5 6 7 8;
          7 8 1 2;
          2 3 6 7;
          1 4 5 8];
      
v_body = [ 1 -0.5 -1.5;
           1  0.5 -1.5;
          -1  0.5 -1.5;
          -1 -0.5 -1.5;
          -1 -0.5  1.5;
          -1  0.5  1.5;
           1  0.5  1.5;
           1 -0.5  1.5].*d;

%% Solar Panels
s = 20;
t = 1;

f_base = [1 2 3 4;
           3 4 5 6;
           5 6 7 8;
           7 8 1 2;
           2 3 6 7;
           1 4 5 8];
v_base = [s -0.5*t -1.5*d;
           s  0.5*t -1.5*d;
           0  0.5*t -1.5*d;
           0 -0.5*t -1.5*d;
           0 -0.5*t  1.5*d;
           0  0.5*t  1.5*d;
           s  0.5*t  1.5*d;
           s -0.5*t  1.5*d];
       
v_45 = (R3(deg2rad(45))*v_base')';
body_offset = repmat([d -0.5*d 0],[8,1]);
v_panel = v_45 + body_offset;
panel_offset = repmat([s/sqrt(2) -s/sqrt(2) 0],[8,1]);

f_panel = [f_base;f_base+8;f_base+16];
v_panel_1 = [v_panel;v_panel+panel_offset;v_panel+2.*panel_offset];
v_panel_2 = (R3(pi)*v_panel_1')';

%% Plot
patch('Faces',f_body,'Vertices',v_body,'FaceColor','red')
hold on
axis equal
patch('Faces',f_panel,'Vertices',v_panel_1,'FaceColor','blue')
patch('Faces',f_panel,'Vertices',v_panel_2,'FaceColor','blue')
view(3)