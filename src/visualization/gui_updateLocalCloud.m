function gui_updateLocalCloud(W_local_cloud, axes_handle, plot_handle)
% Updates 2D plot of local cloud in world frame.
% 
% Input:
%  - W_local_cloud(3xN) : local point cloud in world frame, [x y z]
%  - axes_handle(handle) : gui axes handle
%  - plot_handle(handle) : gui plot handle
%
% Output: none

if ~isempty(W_local_cloud)
    axes(axes_handle);
    hold on;

    % update data to plot
    plot_handle.XData = W_local_cloud(1,:);
    plot_handle.YData = W_local_cloud(2,:);
    axis equal;

    hold off;
end
