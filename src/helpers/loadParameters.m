function params = loadParameters(mode)
% Returns collection of parameters used throughout the VO pipeline.
% 
% Input:
%  - mode(1x1) : parameter setting mode
%
% Output:
%  - params(struct) : parameter struct

% general parameters
params.perf.profiling = false;
params.compare_against_groundthruth = true;

% initialization parameters
params.show_bootstrap_images = false;
params.show_init_landmarks = false;
params.init.use_KITTI_precalculated_init = false;

% continuous operation parameters
params.cont.run_on_first_ten_images = true;

% Correlation parameters
params.corr.show_corr_matches = true;
params.corr.harris_patch_size = 9;
params.corr.harris_kappa = 0.08;
params.corr.nonmaximum_supression_radius = 8;
params.corr.num_keypoints = 200;
params.corr.descriptor_radius = 9;
params.corr.match_lambda = 5;


% params.corr.num_iterations = 200;
% params.corr.pixel_tolerance = 10;
% params.corr.k = 3;


% mode specific parameters
if mode == 1

else
    error('invalid parameter mode');
end

end
