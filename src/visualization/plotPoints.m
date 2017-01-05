function h = plotPoints(keypoints, style)
% Plots 2D points.
% 
% Input:
%  - keypoints(2xN) : 2D points, each [v,u]
%  - style(string) : marker style
%
% Output:
%  - h(handle)

if ~isempty(keypoints)
    assert(size(keypoints,1) == 2,'keypoints have wrong dimensionality');

    h = plot(keypoints(2,:), keypoints(1,:), style, 'Linewidth', 1);
end

end
