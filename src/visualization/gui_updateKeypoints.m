function gui_updateKeypoints(keypoints, axes_handle, style)
% todo: add description
% 
% Input:
%  - keypoints(2xN) : keypoints, each [v u]
%  - img_handle(handle) : gui image handle
%  - style(string) : marker style
%
% Output: none
%
% Note: Always call after updateTrajectory()

axes(axes_handle);

hold on;
plotPoints(keypoints,style);
hold off;
