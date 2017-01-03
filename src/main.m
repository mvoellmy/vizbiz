clear;
close all;
clc;
rng(1); % fix random seed

addpath(genpath('helpers'));
addpath(genpath('testing'));
addpath(genpath('visualization'));

%% Load parameter struct
fprintf('load parameter struct...\n');
params = loadParameters();

%% Setup datasets
if params.ds == 0
    params.kitti_path = '../datasets/kitti';
    assert(isfield(params, 'kitti_path') ~= 0);    
    ground_truth = load([params.kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
    fprintf('load KITTI dataset...\n');
elseif params.ds == 1
    params.malaga_path = '../datasets/malaga-urban-dataset-extract-07';
    assert(isfield(params, 'malaga_path') ~= 0);
    ground_truth = load([params.malaga_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 2121;
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
    fprintf('load MALAGA dataset...\n');
elseif params.ds == 2
    params.parking_path = '../datasets/parking';
    assert(isfield(params, 'parking_path') ~= 0);
    ground_truth = load([params.parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 598;
    K = load([params.parking_path '/K.txt']);
    fprintf('load PARKING dataset...\n');
else
    assert(false);
end

%% Bootstraping
fprintf('setup boostrapping...\n');
tic;
% set bootstrap frames
[img0, img1, bootstrap_frame_idx_1, bootstrap_frame_idx_2] = getBootstrapFrames(params, K);
toc;

fprintf('...boostrapping done.\n\n');

%% Setup logging variables
% set range of images to run on
if (params.cont.run_on_first_x_images > 0)
    range_cont = (bootstrap_frame_idx_2+1):(bootstrap_frame_idx_2+...
             params.cont.run_on_first_x_images);
else
    range_cont = (bootstrap_frame_idx_2+1):last_frame;
end

T_CiCj_vo_j = NaN(4,4,numel(range_cont)+2); % transformation matrix between frame Cj to Ci, range +2 due to init
T_WCj_vo = NaN(4,4,numel(range_cont)+2); % transformation matrix between frame Cj to W, range +2 due to init

%% Code profiling
if params.perf.profiling
    profile on; % trigger code profiling
end

%% Initialize VO pipeline
fprintf('initialize VO pipeline...\n');

global fig_kp_tracks;
fig_kp_tracks = figure('name','Keypoint tracker');

% transformation C1 to W (90deg x-axis rotation)
T_WC1 = [1      0           0       0;
         0      0           1       0;
         0      -1          0       0;
                 zeros(1,3)         1];
             
tic;
% initialize pipeline with bootstrap images
[img_init,keypoints_init,C2_landmarks_init,T_C1C2, kp_tracks] = initPipeline(params,img0,img1,K, T_WC1);
toc;

% normalize scale with ground truth
if params.init.normalize_scale
    [C2_landmarks_init, T_C1C2] = normalizeScale(C2_landmarks_init, T_C1C2, ground_truth, bootstrap_frame_idx_1, bootstrap_frame_idx_2);
end

% assign first two poses
T_CiCj_vo_j(:,:,1) = eye(4); % world frame init, C1 to C1
T_CiCj_vo_j(:,:,2) = T_C1C2; % first camera pose, C2 to C1

% update stacked world-referenced pose
T_WCj_vo(:,:,1) = T_WC1; % C1 to W
T_WCj_vo(:,:,2) = T_WC1*T_C1C2; % C2 to W

% transform init point cloud to world frame
W_P_hom_init = T_WC1*[C2_landmarks_init; zeros(1,size(C2_landmarks_init,2))];
W_landmarks_init = W_P_hom_init(1:3,:);

% full 3D map point cloud in frame W
W_landmarks_map = W_landmarks_init;

% display initialization landmarks and bootstrap motion
if params.init.show_landmarks
    figure('name','Landmarks and motion of bootstrap image pair');
    hold on;
    plotLandmarks(W_landmarks_init,'z','up');
    plotCam(T_WCj_vo(:,:,1),1,'black');
    plotCam(T_WCj_vo(:,:,2),1,'red');
end

fprintf('...initialization done.\n\n');

%% Continuous operation VO pipeline
global fig_cont fig_RANSAC_debug fig_kp_triangulate;

if params.run_continous
    fprintf('start continuous VO operation...\n');

	% setup figure handles
	fig_cont = figure('name','Contiunous VO estimation');
	fig_RANSAC_debug = figure('name','p3p / DLT estimation RANSAC');
    fig_kp_triangulate = figure('name', 'triangulate');

	% hand-over initialization variables
	img_prev = img_init;
    % container for prev kp which have corresponding landmarks
    keypoints_prev_triang = keypoints_init;
    % container for matched keypoints which have yet no corresponding
    % landmark, and the pose where they were seen the first time --> keypoint tracker
        
    % landmarks in last camera frame
	Ci_landmarks_prev = C2_landmarks_init;
    
    for j = range_cont
		fprintf('Processing frame %d\n=====================\n', j);
        frame_idx = j-bootstrap_frame_idx_2+2; % due to init +2

        % pick current frame
        img = getFrame(params, j);
        
        if (size(keypoints_prev_triang,2) > 6) % todo: minimum number?        
            tic;
            % process newest image
            
            % extract current camera pose
            T_WCi = T_WCj_vo(:,:,frame_idx-1); 
            
            [T_CiCj_vo_j(:,:,frame_idx),keypoints_new_triang, updated_kp_tracks,Cj_landmarks_new] =...
                processFrame(params,img,img_prev, keypoints_prev_triang, kp_tracks, Ci_landmarks_prev,T_WCi, K);
            toc;
            
            % add super title with frame number
            figure(fig_cont);
%            suptitle(sprintf('Frame #%i',j));
        else
            warning('Too few keypoints left!! Break continuous operation loop - Terminating...');
            break;
        end

        % append newest Cj to T transformation
        T_WCj_vo(:,:,frame_idx) = T_WCj_vo(:,:,frame_idx-1)*T_CiCj_vo_j(:,:,frame_idx);
        
        % update map with new landmarks
        W_landmarks_new = [];
        if size(Cj_landmarks_new,2)>0
        W_P_hom_new = T_WCj_vo(:,:,frame_idx)*[Cj_landmarks_new; ones(1, size(Cj_landmarks_new,2))];
        W_landmarks_new = W_P_hom_new(1:3,:);
        end
        W_landmarks_map = [W_landmarks_map, W_landmarks_new];

        % allow plots to refresh
        pause(.011);

        % update previous image, keypoints, landmarks and tracker
        img_prev = img;
        keypoints_prev_triang = keypoints_new_triang;
        Ci_landmarks_prev = Cj_landmarks_new;
        kp_tracks = updated_kp_tracks;

        fprintf('\n');
    end
    fprintf('...VO-pipeline terminated.\n');
end

if params.perf.profiling
    profile viewer; % view profiling results
end 

%% Results summary
fprintf('display results...\n');

if (params.ds ~= 1 && params.compare_against_groundthruth)
    % plot VO trajectory against ground truth   
    plotTrajectoryVsGT_2D(T_WCj_vo(1:3,4,:),ground_truth');
elseif (params.ds == 1 && params.compare_against_groundthruth)
    % plot VO trajectory
    plotTrajectory_2D(T_WCj_vo(1:3,4,:));
end

% display full map and cameras
if params.show_map_and_cams
    figure('name','Map landmarks');
    plotLandmarks(W_landmarks_map,'z','up');
    hold on;
    plotCam(T_WCj_vo(:,:,1),2,'black');
    plotCam(T_WCj_vo(:,:,2:end),2,'red');
end
