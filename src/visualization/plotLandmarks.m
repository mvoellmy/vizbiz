function plotLandmarks(landmarks)
% Plots 3D landmarks in world frame.
% 
% Input:
%  - landmarks(3xN) : 3D landmarks in world frame, each [X,Y,Z]
%
% Output: none

assert(size(landmarks,1) == 3,'landmarks have wrong dimensionality');

pc = pointCloud(landmarks');

pcshow(pc, 'VerticalAxis','y','VerticalAxisDir',...
'down','MarkerSize',100);

% axis properties
axis equal;
axis vis3d;
ax = gca;
ax.Projection = 'perspective';

box on;
xlabel('x');
ylabel('y');
zlabel('z');

% Should not be used, since it is dangerous
% Can be enabled to have a reasonable plot until the outliers from parking are fixed.
% xlim([-60 60])
% ylim([-40 40])
% zlim([-10 100])

axis tight

end