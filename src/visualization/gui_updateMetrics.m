function gui_updateMetrics(params, deltaT, metrics_handle)
% todo: add description
% 
% Input:
%  - params(struct) : parameter struct
%  - deltaT(1x1) : computation time in seconds
%  - metrics_handle(handle) : gui metrics handle
%
% Output: none

frame_rate = 30; % todo: through params?

real_time_ness = 100 / (frame_rate * deltaT);
set(metrics_handle, 'String', num2str(real_time_ness, '%.2f'));

end
