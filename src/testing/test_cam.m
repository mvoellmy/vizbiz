clear
clc

addpath(genpath('../visualization/'));


figure
hold on

R_WC1 = [1   0  0;
         0   0  1;
         0  -1  0];
     
r1 = [0 0 0];


R_C1C2 = [0 0 1;
          0 1 0;
          -1 0 0;];
      
R_WC2 = R_WC1*R_C1C2;

r2 = [0 0 0];


      
plotCamera('Location', r1, 'Orientation', R_WC1', 'size', 0.8)
plotCamera('Location', r2, 'Orientation', R_WC2','color','black');

% plotCamera('Location', [0 0 0], 'Orientation', eye(3))

% plotCam([R_WC1, r';zeros(1,4);], 0.5, 'blue')

axis tight
axis equal;
axis vis3d;
ax = gca;
ax.Projection = 'perspective';

xlabel('X');
ylabel('Y')
zlabel('Z')