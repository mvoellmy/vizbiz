function plotPoints(keypoints)
% Plots 2D points.
% 
% Input:
%  - keypoints(Nx2) : 2D points, each [u,v]
%
% Output: none

assert(size(keypoints,2) == 2,'keypoints have wrong dimensionality');
plot(keypoints(:,1), keypoints(:,2), 'rx', 'Linewidth', 1);

end