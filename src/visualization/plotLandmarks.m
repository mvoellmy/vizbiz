function plotLandmarks(landmarks)
% Plots 3D landmarks in world frame.
% 
% Input:
%  - landmarks(3xN) : 3D landmarks in world frame, each [X,Y,Z]
%
% Output: none

assert(size(landmarks,1) == 3,'landmarks have wrong dimensionality');

scatter3(landmarks(1,:), landmarks(2,:), landmarks(3,:), ...
         20*ones(1,length(landmarks)), 'red', 'filled');
axis equal;
axis vis3d;
grid off;
xlabel('X');
ylabel('Y');
zlabel('Z');

end