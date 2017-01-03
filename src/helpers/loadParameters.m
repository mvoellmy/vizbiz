function params = loadParameters()
% Returns collection of parameters used throughout the VO pipeline.
% 
% Input: none
%
% Output:
%  - params(struct) : parameter struct

% general parameters
params.ds = 2; % 0: KITTI, 1: Malaga, 2: Parking
params.auto_bootstrap = false;
params.perf.profiling = false;
params.compare_against_groundthruth = true;
params.run_continous = true;
params.run_on_first_x_images = 50; % 0 for all images
params.show_map_and_cams = true;
params.through_gui = false;

% additional gui parameters
params.gui.show_all_features = true;
params.gui.show_inlier_features = true;

% bootstrap parameters
params.boot.figures = true; % on/off figure
params.boot.show_boot_images = true;
params.boot.num_keypoints = 600;
params.boot.show_boot_keypoints = true;
params.boot.show_matches = true;
params.boot.landmarks_cutoff = 100;
params.boot.show_boot_landmarks = true;
params.boot.min_num_inlier_kps = 100;
params.boot.min_b2dratio = 0.1;

% initialization parameters
params.init.figures = true; % on/off figure
params.init.show_keypoints = false;
params.init.show_inlier_matches = true;
params.init.show_landmarks = false;
params.init.use_KITTI_precalculated_init = false;
params.init.show_matches = true;
params.init.use_BA = true;
params.init.landmarks_cutoff = 100;
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
params.eightPoint_ransac.max_error = 1.0; % [pixels]

% continuous operation parameters
params.cont.figures = true; % on/off figure
params.cont.show_new_image = true;
params.cont.show_new_keypoints = true;
params.cont.use_KLT = true;
params.cont.show_matches = true;
params.cont.show_inlier_matches = true;
params.cont.landmarks_cutoff = 5;

params.localization_ransac.show_matched_keypoints = true;
params.localization_ransac.show_inlier_matches = true;
params.localization_ransac.use_p3p = true;
params.localization_ransac.num_iterations_pnp = 3000; % 2000 fix?
params.localization_ransac.num_iterations_DLT = 200;
params.localization_ransac.pixel_tolerance = 10; % 10 [pixels]
params.localization_ransac.show_iterations = false;

params.kp_tracker.show_matches = true;
params.kp_tracker.show_triangulated = true;
params.kp_tracker.bearing_low_thr = 3.2; % [deg]
params.kp_tracker.bearing_up_thr = params.kp_tracker.bearing_low_thr*2.5; % [deg]
params.kp_tracker.min_nr_trackings = 2; % 3
params.kp_tracker.max_reproj_error = 10; % 12 [pixels]

end
