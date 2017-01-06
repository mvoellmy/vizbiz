function gui_updateGT(ground_truth, axes_handle, plot_handle)
% Updates ground truth path provided by dataset.
% 
% Input:
%  - W_trajectory(2xN) : 2D trajectory in world frame, each [x y]
%  - axes_handle(handle) : gui axes handle
%  - plot_handle(handle) : gui plot handle
%
% Output: none

% update data to plot
axes(axes_handle);
plot_handle.XData = ground_truth(:,1);
plot_handle.YData = ground_truth(:,2);

axis equal;
