function params = loadParameters(mode)
% Returns collection of parameters used throughout the VO pipeline.
% 
% Input:
%  - mode(1x1) : parameter setting mode
%
% Output:
%  - params(struct) : parameter struct

% general parameters
params.ds = 0; % 0: KITTI, 1: Malaga, 2: Parking
params.perf.profiling = false;
params.compare_against_groundthruth = true;

% initialization parameters
params.init.show_bootstrap_images = false;
params.init.show_landmarks = false;
params.init.use_KITTI_precalculated_init = false;
params.init.show_corr_matches = true;

% Correlation parameters
params.corr.harris_patch_size = 9;
params.corr.harris_kappa = 0.08;
params.corr.nonmaximum_supression_radius = 8;
params.corr.num_keypoints = 400; % 200
params.corr.descriptor_radius = 9;
params.corr.match_lambda = 6; % 5

params.eightPoint_ransac.show_iterations = true;
params.eightPoint_ransac.show_inlier_matches = true;
params.eightPoint_ransac.p_success = 0.9999;
params.eightPoint_ransac.fract_inliers = 0.5;
params.eightPoint_ransac.max_error = 1;

% continuous operation parameters
params.cont.run_on_first_ten_images = true;
params.cont.show_current_image = true;
params.cont.show_new_keypoints = true;
params.cont.show_matches = false;

params.localization_ransac.show_matched_keypoints = true;
params.localization_ransac.show_inlier_matches = true;

params.localization_ransac.use_p3p = true;



% mode specific parameters
if mode == 1

else
    error('invalid parameter mode');
end

end
