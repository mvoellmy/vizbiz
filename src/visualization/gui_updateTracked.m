function gui_updateTracked(params, bar_length, metrics_handle, axes_handle, plot_handle)
% Updates indicator track bar.
% 
% Input:
%  - params(struct) : parameter struct
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

% adapt bar color
if bar_length > params.kp_tracker.min_nr_landmarks_bearing_angle_adapt
    set(plot_handle,'Color',[0.5 0.5 0.5]);
else
    set(plot_handle,'Color',[0.8 0 0]);
end

xlim([0 400]); % todo: parametrize?
axis equal;
axis off;

% update metric
set(metrics_handle, 'String', num2str(bar_length, '%i'));
