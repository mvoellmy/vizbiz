function plotCircles(keypoints, style, radius)
% Plots circles at 2D points.
% 
% Input:
%  - keypoints(2xN) : 2D points, each [v,u]
%  - style(string) : marker style
%  - radius(1x1) : circle radius in pixels
%
% Output: none

assert(size(keypoints,1) == 2,'keypoints have wrong dimensionality');

ang = 0:0.01:2*pi;
for i=1:size(keypoints,2)
    plot(keypoints(2,i)+radius*cos(ang), keypoints(1,i)+radius*sin(ang), style);
end
    
end
