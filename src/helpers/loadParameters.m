function p = loadParameters(mode)
% Returns collection of parameters used throughout the VO pipeline
% 
% Input: point correspondences
%  - mode(scalar) : parameter setting mode
%
% Output:
%  - p(struct) : parameter struct

% general parameters
p.perf.profiling = false;

% initialization parameters
p.show_bootstrap_images = false;
p.show_init_images = true;
p.init.use_KITTI_precalculated_init = true;

% continuous operation parameters
p.cont.run_on_first_ten_images = true;

% mode specific parameters
if mode == 1
    
else
    error('invalid parameter mode');
end


end