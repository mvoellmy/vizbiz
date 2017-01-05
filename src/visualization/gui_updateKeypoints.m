function gui_updateKeypoints(keypoints, axes_handle, style)
% Updates keypoints on top of current frame.
% 
% Input:
%  - keypoints(2xN) : keypoints, each [v u]
%  - img_handle(handle) : gui image handle
%  - style(string) : marker style
%
% Output: none
%
% Note: always call after updateImage()

axes(axes_handle);

hold on;
plotPoints(keypoints,style);
hold off;
