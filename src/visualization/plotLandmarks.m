function plotLandmarks(landmarks)
% Some nice function description. TODO
% 
% Input:
%  - landmarks(3xN) : 3D landmarks in world frame
%
% Output: none

scatter3(landmarks(1,:), landmarks(2,:), landmarks(3,:), ...
         20 * ones(1, length(landmarks)), 'red', 'filled');
axis equal;
axis vis3d;
grid off;
xlabel('X');
ylabel('Y');
zlabel('Z');

end