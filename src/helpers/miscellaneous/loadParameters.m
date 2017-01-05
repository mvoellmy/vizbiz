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

params.run_on_first_x_images = 0; % 0 for all images
params.show_map_and_cams = true;
params.through_gui = false;

% additional gui parameters
params.gui.show_all_features = false;
params.gui.show_inlier_features = false;
params.gui.show_triang_features = false;

% bootstrap parameters
params.boot.figures = false; % on/off figure
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
params.init.use_KLT = true; % --------------------------------------
params.init.show_keypoints = true;
params.init.show_inlier_matches = true;
params.init.show_landmarks = true;
params.init.use_KITTI_precalculated_init = false;
params.init.show_matches = true;
params.init.use_BA = false;
params.init.landmarks_cutoff = 100; % --------------------------------------
params.init.normalize_scale = true;

% correspondence parameters initialisation
params.init.corr.harris_patch_size = 9; % 9 [pixels]
params.init.corr.harris_kappa = 0.08;
params.init.corr.nonmaximum_supression_radius = 8;
params.init.corr.num_keypoints = 600; % 200 % 400 % --------------------------------------
params.init.corr.descriptor_radius = 9; % [pixels]
params.init.corr.match_lambda = 8; % 5 % --------------------------------------

params.eightPoint_ransac.show_iterations = false;
params.eightPoint_ransac.show_inlier_matches = true;
params.eightPoint_ransac.p_success = 0.999999;
params.eightPoint_ransac.fract_inliers = 0.5;
params.eightPoint_ransac.max_error = 1.0; % [pixels]

% continuous operation parameters
params.cont.figures = true; % on/off figure
params.cont.show_new_image = true;
params.cont.show_new_keypoints = true;
params.cont.use_KLT = true;   % --------------------------------------
params.cont.show_matches = true;
params.cont.show_inlier_matches = true;
params.cont.landmarks_cutoff = 100;  % --------------------------------------
params.cont.plot_new_landmarks = false;

% correspondence parameters continiuous
params.cont.corr.harris_patch_size = 9; % 9 [pixels]
params.cont.corr.harris_kappa = 0.08;
params.cont.corr.nonmaximum_supression_radius = 8;
params.cont.corr.num_keypoints = 200; % 200 % --------------------------------------
params.cont.corr.descriptor_radius = 9; % [pixels]
params.cont.corr.match_lambda = 6; % 5 % --------------------------------------

params.localization_ransac.show_matched_keypoints = true;
params.localization_ransac.show_inlier_matches = true;
params.localization_ransac.use_p3p = true;
params.localization_ransac.num_iterations_pnp = 2000; % 2000 fix?
params.localization_ransac.num_iterations_DLT = 200;
params.localization_ransac.pixel_tolerance = 8; % 10 [pixels]  % --------------------------------------
params.localization_ransac.show_iterations = false;

params.kp_tracker.use_KLT = true;  % --------------------------------------
params.kp_tracker.min_nr_landmarks = 250;
params.kp_tracker.min_nr_landmarks_bearing_angle_adapt = 150;
params.kp_tracker.bearing_angle_multiplicator = 2.5;
params.kp_tracker.max_nr_candidates = 600;  % --------------------------------------
params.kp_tracker.show_matches = true;
params.kp_tracker.show_triangulated = true;
params.kp_tracker.bearing_low_thr = 1.2; % [deg]  % --------------------------------------
params.kp_tracker.bearing_up_thr = params.kp_tracker.bearing_low_thr*3.5; % [deg]
params.kp_tracker.min_nr_trackings = 2; % 3
params.kp_tracker.max_nr_trackings = 20;
params.kp_tracker.max_reproj_error = 10; % 12 [pixels]  % --------------------------------------

end
