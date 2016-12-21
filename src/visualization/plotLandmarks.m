function plotLandmarks(landmarks, verticalAxis, verticalDir)
% Plots landmarks in 3D space. By aligning verticalAxis (z) 'up' the
% world-frame reference is chosen. With (y) 'down' the cam-frame is used.
% 
% Input:
%  - landmarks(3xN) : 3D landmarks, each [X,Y,Z]
%
% Output: none

assert(size(landmarks,1) == 3,'landmarks have wrong dimensionality');

pc = pointCloud(landmarks');
pcshow(pc,'VerticalAxis',verticalAxis,'VerticalAxisDir',verticalDir,'MarkerSize',100);

% axis properties

% Should not be used, since it is dangerous
% Can be enabled to have a reasonable plot until the outliers from parking are fixed.
% xlim([-60 60])
% ylim([-40 40])
% zlim([-10 100])

axis tight
axis equal;
axis vis3d;
ax = gca;
ax.Projection = 'perspective';

box on;
grid on
xlabel('x');
ylabel('y');
zlabel('z');

end
