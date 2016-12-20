function updateTrajectory(W_trajectory, axes_handle, plot_handle)
% todo: add description
% 
% Input:
%  - W_trajectory(2xN) : 2D trajectory in world frame, each [x y]
%  - axes_handle(handle) : gui axes handle
%  - plot_handle(handle) : gui plot handle
%
% Output: none

axes(axes_handle);

% update data to plot
plot_handle.XData = W_trajectory(1,:);
plot_handle.YData = W_trajectory(2,:);

w = 100; % window size
x_last = W_trajectory(1,end);
y_last = W_trajectory(2,end);

axis([x_last-w/2 x_last+w/2 y_last-w/2 y_last+w/2]);
axis equal;
axis square;
