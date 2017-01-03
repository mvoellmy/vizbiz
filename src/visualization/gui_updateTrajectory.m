function gui_updateTrajectory(W_trajectory, axes_handle, plot_handle)
% todo: add description
% 
% Input:
%  - W_trajectory(2xN) : 2D trajectory in world frame, each [x y]
%  - axes_handle(handle) : gui axes handle
%  - plot_handle(handle) : gui plot handle
%
% Output: none

% update data to plot
axes(axes_handle);
plot_handle.XData = W_trajectory(1,:);
plot_handle.YData = W_trajectory(2,:);

w = 100; % window size, todo: parametrize?
x_last = W_trajectory(1,end);
y_last = W_trajectory(2,end);

axis equal;
axis([x_last-w/2 x_last+w/2 y_last-w*1/3 y_last+w*2/3]);
