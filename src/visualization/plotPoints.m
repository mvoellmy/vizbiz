function plotPoints(keypoints)
% Plots 2D points.
% 
% Input:
%  - keypoints(2xN) : 2D points, each [v,u]
%
% Output: none

assert(size(keypoints,1) == 2,'keypoints have wrong dimensionality');
plot(keypoints(2,:), keypoints(1,:), 'rx', 'Linewidth', 1);

end