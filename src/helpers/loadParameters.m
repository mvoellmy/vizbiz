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
p.compare_against_groundthruth = true;

% initialization parameters
p.show_bootstrap_images = false;
p.show_init_images = true;
p.init.use_KITTI_precalculated_init = false;

% continuous operation parameters
p.cont.run_on_first_ten_images = true;

% Correlation parameters
p.corr.harris_patch_size = 9;
p.corr.harris_kappa = 0.08;
p.corr.nonmaximum_supression_radius = 8;
p.corr.descriptor_radius = 9;
p.corr.match_lambda = 4;

p.corr.num_keypoints = 200;

p.corr.num_iterations = 200;
p.corr.pixel_tolerance = 10;
p.corr.k = 3;


% mode specific parameters
if mode == 1

else
    error('invalid parameter mode');
end


end