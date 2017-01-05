function gui_updateMetrics(deltaT, metrics_handle)
% Updates real-time-ness indicator.
% 
% Input:
%  - deltaT(1x1) : computation time in seconds
%  - metrics_handle(handle) : gui metrics handle
%
% Output: none

frame_rate = 10; % adapted from Kitti dataset

real_time_ness = 100 / (frame_rate * deltaT);
set(metrics_handle, 'String', num2str(real_time_ness, '%.2f'));

end
