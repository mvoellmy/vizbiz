clear
clc

figure
% plotCamera('Location', [0 0 0], 'Orientation', [1 0 0; 0 0 -1; 0 1 0])
plotCamera('Location', [0 0 0], 'Orientation', eye(3))

xlabel('X');
ylabel('Y')
zlabel('Z')