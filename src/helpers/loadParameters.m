function params = loadParameters()
% Returns collection of parameters used throughout the VO pipeline.
% 

% Output:
%  - params(struct) : parameter struct

% general parameters
params.ds = 2; % 0: KITTI, 1: Malaga, 2: Parking
params.auto_bootstrap = false;
params.perf.profiling = false;
params.compare_against_groundthruth = true;
params.run_continous = true;
params.show_map_and_cams = true;


% bootstrap parameters
params.boot.show_bootstrap_images = true;
params.boot.num_keypoints = 600;
params.boot.show_boot_keypoints = true;
params.boot.show_corr_matches = true;
params.boot.landmarks_cutoff = 100;
params.boot.show_boot_landmarks = false;
params.boot.min_num_inlier_kps = 100;
params.boot.min_b2dratio = 0.1;

% initialization parameters
params.init.show_init_keypoints = true;
params.init.show_landmarks = true;
params.init.use_KITTI_precalculated_init = false;
params.init.show_corr_matches = true;
params.init.use_BA = false;
params.init.landmarks_cutoff = 50;
params.init.normalize_scale = true;

% correspondence parameters
params.corr.harris_patch_size = 9; % 9 [pixels]
params.corr.harris_kappa = 0.08;
params.corr.nonmaximum_supression_radius = 8;
params.corr.num_keypoints = 400; % 200
params.corr.descriptor_radius = 9; % [pixels]
params.corr.match_lambda = 6; % 5

params.eightPoint_ransac.show_iterations = false;
params.eightPoint_ransac.show_inlier_matches = true;
params.eightPoint_ransac.p_success = 0.999999;
params.eightPoint_ransac.fract_inliers = 0.5;
params.eightPoint_ransac.max_error = 1;
params.eightPoint_ransac_cont.max_error = 1;

% continuous operation parameters

params.cont.run_on_first_x_images = 40; % 0 for all images
params.cont.show_current_image = true;
params.cont.show_new_keypoints = true;
params.cont.show_matches = true;
params.cont.landmarks_cutoff = 5;

params.localization_ransac.show_matched_keypoints = true;
params.localization_ransac.show_inlier_matches = false;

params.localization_ransac.use_p3p = true;
params.localization_ransac.num_iterations_pnp = 3000; % 2000 fix?
params.localization_ransac.num_iterations_DLT = 200;
params.localization_ransac.pixel_tolerance = 10; % 10 [pixels]
params.localization_ransac.show_iterations = true;

params.keypoint_tracker.show_matches = true;
params.keypoint_tracker.show_triangulated = true;
params.keypoint_tracker.bearing_low_thr = 3.2;
params.keypoint_tracker.bearing_up_thr = params.keypoint_tracker.bearing_low_thr*2.5;
params.keypoint_tracker.min_nr_trackings = 3;
params.keypoint_tracker.max_reproj_error = 12; % [pixels]

end
