function plotLandmarks(landmarks)
% Plots 3D landmarks in world frame.
% 
% Input:
%  - landmarks(3xN) : 3D landmarks in world frame, each [X,Y,Z]
%
% Output: none

assert(size(landmarks,1) == 3,'landmarks have wrong dimensionality');

scatter3(landmarks(1,:), landmarks(2,:), landmarks(3,:), ...
         10*ones(1,size(landmarks,2)), 'red', 'filled');
     
% axis properties
axis equal;
axis vis3d;
ax = gca;
ax.Projection = 'perspective';

box on;
xlabel('x');
ylabel('y');
zlabel('z');

end