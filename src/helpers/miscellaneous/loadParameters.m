function params = loadParameters()
% Returns collection of parameters used throughout the VO pipeline.
% 
% Input: none
%
% Output:
%  - params(struct) : parameter struct


%% general parameters
% general parameters
params.ds = 0; % 0: KITTI, 1: Malaga, 2: Parking, 3: Poly-up, 4: Poly-down
params.auto_bootstrap = false;
params.perf.profiling = false;
params.run_continous = true;
params.run_on_first_x_images = 60; % 0 for all images

params.compare_against_groundthruth = true;
params.show_map_and_cams = true;
params.through_gui = false;


% additional gui parameters
params.gui.show_all_features = false;
params.gui.show_inlier_features = false;
params.gui.show_triang_features = false;

% auto bootstrap parameters
params.boot.figures = false; % on/off figure
params.boot.use_bearing_angle = true; % approach: 2
params.boot.num_keypoints = 1000;
params.boot.show_keypoints = true;
params.boot.show_matches = true;
params.boot.show_inlier_matches = true;
params.boot.landmarks_cutoff = 500;
params.boot.show_landmarks = false;
params.boot.min_num_inlier_kps = 600; % 600
params.boot.min_b2dratio = 0.1;
params.boot.min_av_angle_deg = 10; % [deg]

% initialization parameters
params.init.figures = true; % on/off figure
params.init.use_KLT = true; % --------------------------------------
params.init.show_keypoints = true;
params.init.show_inlier_matches = true;
params.init.show_landmarks = true;
params.init.use_KITTI_precalculated_init = false;
params.init.show_matches = true;
params.init.use_BA = true;
params.init.show_BA_comp = false;

params.init.normalize_scale = true;

% correspondence parameters initialisation
params.init.corr.harris_patch_size = 9; % 9 [pixels]
params.init.corr.harris_kappa = 0.08; %0.08;
params.init.corr.nonmaximum_supression_radius = 8;
params.init.corr.num_keypoints = 600; % 200 % 400 % --------------------------------------
params.init.corr.descriptor_radius = 9; % [pixels]
params.init.corr.match_lambda = 8; % 5 % --------------------------------------

params.eightPoint_ransac.show_iterations = false;
params.eightPoint_ransac.show_inlier_matches = true;
params.eightPoint_ransac.p_success = 0.999999;
params.eightPoint_ransac.fract_inliers = 0.5;
params.eightPoint_ransac.max_error = 2.0; % [pixels]

% continuous operation parameters
params.cont.figures = true; % on/off figure
params.cont.show_new_image = true;
params.cont.show_new_keypoints = true;
params.cont.use_KLT = true;   % --------------------------------------
params.cont.show_matches = true;
params.cont.show_inlier_matches = true;
params.cont.plot_new_landmarks = false;

% bundle adjustment
params.cont.use_BA = true;
params.cont.ba.frequency = 10;
params.cont.ba.fix_view_ids = false;
params.cont.ba.window_size = 15;

% correspondence parameters continiuous
params.cont.corr.harris_patch_size = 9; % 9 [pixels]
params.cont.corr.harris_kappa = 0.08;
params.cont.corr.nonmaximum_supression_radius = 8;
params.cont.corr.num_keypoints = 200; % 200 % --------------------------------------
params.cont.corr.descriptor_radius = 9; % [pixels]
params.cont.corr.match_lambda = 6; % 5 % --------------------------------------

% Reinitialization parameters
params.cont.reinit.do_reinit = true;

params.localization_ransac.show_matched_keypoints = true;
params.localization_ransac.show_inlier_matches = true;
params.localization_ransac.use_p3p = true;
params.localization_ransac.num_iterations_DLT = 200;
params.localization_ransac.show_iterations = false;

params.kp_tracker.figures = false;
params.kp_tracker.show_matches = true;
params.kp_tracker.show_triangulated = true;
params.kp_tracker.use_KLT = true;  % --------------------------------------

%% database dependent parameters

% Kitti
if params.ds == 0
    params.init.landmarks_cutoff = 200; % --------------------------------------
    
    params.cont.reinit.inlier_th = 50; % when to reinit
    params.cont.reinit.deltaFrames = 3;
    
    params.cont.landmarks_cutoff = 100;  % --------------------------------------
    
    params.localization_ransac.num_iterations_pnp = 1500; % 2000 fix?
    params.localization_ransac.pixel_tolerance = 10; % 10 [pixels]  % -------------------------------------
    
    params.kp_tracker.min_nr_landmarks = 500;
    params.kp_tracker.min_nr_new_landmarks = 200;
    params.kp_tracker.min_nr_landmarks_bearing_angle_adapt = 230;
    params.kp_tracker.bearing_angle_multiplicator = 1.75;
    params.kp_tracker.max_nr_candidates = 1200;  % --------------------------------------
    params.kp_tracker.rand_pick = false;
    params.kp_tracker.nr_best_candidates = 100; % for randomized picking
    params.kp_tracker.bearing_low_thr = 6; % [deg]  % --------------------------------------
    params.kp_tracker.bearing_up_thr = params.kp_tracker.bearing_low_thr*10.5; % [deg]
    params.kp_tracker.min_nr_trackings = 2; % 3
    params.kp_tracker.max_nr_trackings = 20;
    params.kp_tracker.max_reproj_error = 7; % 12 [pixels]  % --------------------------------------

% Malaga
elseif params.ds == 1
    params.init.landmarks_cutoff = 200; % --------------------------------------
    
    params.cont.reinit.inlier_th = 130; % when to reinit
    params.cont.reinit.deltaFrames = 3;
    
    params.cont.landmarks_cutoff = 100;  % --------------------------------------
    
    params.localization_ransac.num_iterations_pnp = 2000; % 2000 fix?
    params.localization_ransac.pixel_tolerance = 10; % 10 [pixels]  % -------------------------------------
    
    params.kp_tracker.min_nr_landmarks = 500;
    params.kp_tracker.min_nr_new_landmarks = 200;
    params.kp_tracker.min_nr_landmarks_bearing_angle_adapt = 230;
    params.kp_tracker.bearing_angle_multiplicator = 1.75;
    params.kp_tracker.max_nr_candidates = 1100;  % --------------------------------------
    params.kp_tracker.rand_pick = false;
    params.kp_tracker.nr_best_candidates = 100; % for randomized picking
    params.kp_tracker.bearing_low_thr = 7; % [deg]  % --------------------------------------
    params.kp_tracker.bearing_up_thr = params.kp_tracker.bearing_low_thr*10.5; % [deg]
    params.kp_tracker.min_nr_trackings = 2; % 3
    params.kp_tracker.max_nr_trackings = 20;
    params.kp_tracker.max_reproj_error = 7; % 12 [pixels]  % --------------------------------------

% Parking
elseif params.ds == 2
    params.init.landmarks_cutoff = 200; % --------------------------------------
    
    params.cont.reinit.inlier_th = 80; % when to reinit
    params.cont.reinit.deltaFrames = 3;
    
    params.cont.landmarks_cutoff = 370;  % --------------------------------------
    
    params.localization_ransac.num_iterations_pnp = 400; % 2000 fix?
    params.localization_ransac.pixel_tolerance = 2; % 10 [pixels]  % -------------------------------------
    
    params.kp_tracker.min_nr_landmarks = 500; % this is the number desired
    params.kp_tracker.min_nr_landmarks_bearing_angle_adapt = 230;
    params.kp_tracker.bearing_angle_multiplicator = 2.5;
    params.kp_tracker.max_nr_candidates = 1300;  % --------------------------------------
    params.kp_tracker.rand_pick = false;
    params.kp_tracker.nr_best_candidates = 100; % for randomized picking
    params.kp_tracker.bearing_low_thr = 3.7; % [deg]  % --------------------------------------
    params.kp_tracker.bearing_up_thr = params.kp_tracker.bearing_low_thr*10.5; % [deg]
    params.kp_tracker.min_nr_trackings = 3; % 3
    params.kp_tracker.max_nr_trackings = 25;
    params.kp_tracker.max_reproj_error = 2; % 12 [pixels]  % -------------------------------------- 

% Poly-Up
elseif params.ds == 3
    params.init.landmarks_cutoff = 200; % --------------------------------------
    
    params.cont.reinit.inlier_th = 150; % when to reinit
    params.cont.reinit.deltaFrames = 3;
    
    params.cont.landmarks_cutoff = 100;  % --------------------------------------
    
    params.localization_ransac.num_iterations_pnp = 2000; % 2000 fix?
    params.localization_ransac.pixel_tolerance = 7; % 10 [pixels]  % -------------------------------------
    
    params.kp_tracker.min_nr_landmarks = 500;
    params.kp_tracker.min_nr_new_landmarks = 200;
    params.kp_tracker.min_nr_landmarks_bearing_angle_adapt = 250;
    params.kp_tracker.bearing_angle_multiplicator = 1.75;
    params.kp_tracker.max_nr_candidates = 1300;  % --------------------------------------
    params.kp_tracker.rand_pick = false;
    params.kp_tracker.nr_best_candidates = 100; % for randomized picking
    params.kp_tracker.bearing_low_thr = 5; % [deg]  % --------------------------------------
    params.kp_tracker.bearing_up_thr = params.kp_tracker.bearing_low_thr*10.5; % [deg]
    params.kp_tracker.min_nr_trackings = 2; % 3
    params.kp_tracker.max_nr_trackings = 10;
    params.kp_tracker.max_reproj_error = 7; % 12 [pixels]  % -------------------------------------- 
    
% Poly-Down
elseif params.ds == 4
    params.init.landmarks_cutoff = 200; % --------------------------------------
    
    params.cont.reinit.inlier_th = 150; % when to reinit
    params.cont.reinit.deltaFrames = 3;
    
    params.cont.landmarks_cutoff = 100;  % --------------------------------------
    
    params.localization_ransac.num_iterations_pnp = 2000; % 2000 fix?
    params.localization_ransac.pixel_tolerance = 7; % 10 [pixels]  % -------------------------------------
    
    params.kp_tracker.min_nr_landmarks = 500;
    params.kp_tracker.min_nr_landmarks_bearing_angle_adapt = 250;
    params.kp_tracker.bearing_angle_multiplicator = 1.75;
    params.kp_tracker.max_nr_candidates = 1300;  % --------------------------------------
    params.kp_tracker.rand_pick = false;
    params.kp_tracker.nr_best_candidates = 100; % for randomized picking
    params.kp_tracker.bearing_low_thr = 5; % [deg]  % --------------------------------------
    params.kp_tracker.bearing_up_thr = params.kp_tracker.bearing_low_thr*10.5; % [deg]
    params.kp_tracker.min_nr_trackings = 2; % 3
    params.kp_tracker.max_nr_trackings = 10;
    params.kp_tracker.max_reproj_error = 7; % 12 [pixels]  % -------------------------------------- 
    
else
    error('!!!!! Wrong dataset parameter !!!!!!')
end
