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
p.init.use_KITTI_precalculated_init = true;

% continuous operation parameters
p.cont.run_on_first_ten_images = true;

% mode specific parameters
if mode == 1
%     harris_patch_size = 9;
%     harris_kappa = 0.08;
%     nonmaximum_supression_radius = 8;
%     descriptor_radius = 9;
%     match_lambda = 5;

%     num_keypoints = 1000;

%     num_iterations = 200;
%     pixel_tolerance = 10;
%     k = 3;
else
    error('invalid parameter mode');
end


end