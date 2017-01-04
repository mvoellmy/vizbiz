function gui_updateTracked(bar_length, metrics_handle, axes_handle, plot_handle)
% Updates indicator track bar.
% 
% Input:
%  - bar_length(1x1) : bar length, integer
%  - metrics_handle(handle) : gui metrics handle
%  - axes_handle(handle) : gui axes handle
%  - plot_handle(handle) : gui plot handle
%
% Output: none

% update bar plot
axes(axes_handle);
plot_handle.YData = [0, 0];
plot_handle.XData = [1, bar_length];

xlim([0 200]);
axis equal;
axis off;

% update metric
set(metrics_handle, 'String', num2str(bar_length, '%i'));
