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
    kitti_path = '../datasets/kitti';
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
    fprintf('load KITTI dataset...\n');
elseif params.ds == 1
    malaga_path = '../datasets/malaga-urban-dataset-extract-07';
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
    fprintf('load MALAGA dataset...\n');
elseif params.ds == 2
    parking_path = '../datasets/parking';
    assert(exist('parking_path', 'var') ~= 0);     
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
    fprintf('load PARKING dataset...\n');
else
    assert(false);
end

%% Bootstraping
fprintf('setup boostrapping...\n\n');
% set bootstrap_frames
if params.ds == 0
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrapFrames(params.ds,'first'))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrapFrames(params.ds,'second'))]);
elseif params.ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrapFrames(params.ds,'first')).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrapFrames(params.ds,'second')).name]));
elseif params.ds == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrapFrames(params.ds,'first'))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrapFrames(params.ds,'second'))]));
else
    assert(false);
end

% display boostrap images
if params.init.show_bootstrap_images    
    figure('name','Boostrap images');
    subplot(1,2,1);
    imshow(img0);
    subplot(1,2,2);
    imshow(img1);
end

%% Logging variables
% set range of images to run on
bootstrap_frame_idx_2 = bootstrapFrames(params.ds,'second');
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

% transformation C1 to W (90deg x-axis rotation)
T_WC1 = [1      0           0       0;
         0      0           1       0;
         0     -1           0       0;
                 zeros(1,3)         1];

tic;
% initialize pipeline with bootstrap images
[img_init,keypoints_init,C2_landmarks_init,T_C1C2] = initPipeline(params,img0,img1,K, T_WC1);
toc;

% assign first two poses
T_CiCj_vo_j(:,:,1) = eye(4); % world frame init, C1 to C1
T_CiCj_vo_j(:,:,2) = T_C1C2; % first camera pose, C2 to C1

% update stacked world-referenced pose
T_WCj_vo(:,:,1) = T_WC1; % C1 to W
T_WCj_vo(:,:,2) = T_WC1*T_C1C2; % C2 to W

% transform init point cloud to world frame
W_P_hom_init = T_WC1*[C2_landmarks_init; zeros(1,size(C2_landmarks_init,2))];
W_landmarks_init = W_P_hom_init(1:3,:);
W_landmarks_map = W_landmarks_init; % full 3D map point cloud in frame W

% display initialization landmarks and bootstrap motion
if params.init.show_landmarks
    figure('name','Landmarks and motion of bootstrap image pair');
    hold on;
    plotLandmarks(W_landmarks_init,'z','up');
    plotCam(T_WCj_vo(:,:,1),2,'black');
    plotCam(T_WCj_vo(:,:,2),2,'red');
end

fprintf('...initialization done.\n\n');

%% Continuous operation VO pipeline
global fig_cont fig_kp_tracks fig_RANSAC_debug;

if params.run_continous
    fprintf('start continuous VO operation...\n');

	% setup figure handles
	fig_cont = figure('name','Contiunous VO estimation');
	fig_RANSAC_debug = figure('name','p3p / DLT estimation RANSAC');
    fig_kp_tracks = figure('name','Keypoint tracker');

	% hand-over initialization variables
	img_prev = img_init;
    % container for prev kp which have corresponding landmarks
    keypoints_prev_triang = keypoints_init;
    % container for matched keypoints which have yet no corresponding
    % landmark, and the pose where they were seen the first time --> keypoint tracker
    
    % Create container for keypoint tracker (new keypoints with no landmarks)
    % set of candidate keypoints in last camera frame
    kp_tracks.candidate_kp = []; % 2xN
    % keypoint coordinates of every candiate in its first observed frame
    kp_tracks.first_obs_kp = [];  % 2xN
    % keypoint pose of every candiate in its first observed frame
    kp_tracks.first_obs_pose = []; % 16xN
    
    % landmarks in last camera frame
	Ci_landmarks_prev = C2_landmarks_init;
    
    % unused?
	%match_indices_prev = 1:size(keypoints_prev_triang,2); 

    for j = range_cont
		fprintf('Processing frame %d\n=====================\n', j);
        frame_idx = j-bootstrap_frame_idx_2+2; % due to init +2
        
        if params.ds == 0
            img = imread([kitti_path '/00/image_0/' sprintf('%06d.png',j)]);
        elseif params.ds == 1
            img = rgb2gray(imread([malaga_path ...
                '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
                left_images(j).name]));
        elseif params.ds == 2
            img = im2uint8(rgb2gray(imread([parking_path ...
                sprintf('/images/img_%05d.png',j)])));
        else
            assert(false);
        end

        if (size(keypoints_prev_triang,2) > 10) % todo: minimum number?        
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
        
        % TODO: Check it!
        % update map with new landmarks
        W_P_hom_new = T_WCj_vo(:,:,frame_idx)*[Cj_landmarks_new; ones(1, size(Cj_landmarks_new,2))];
        W_landmarks_new = W_P_hom_new(1:3,:);
        W_landmarks_map = [W_landmarks_map W_landmarks_new];

        % allow plots to refresh
        pause(1.01);

        % update previous image, keypoints, landmarks and tracker
        img_prev = img;
        keypoints_prev_triang = keypoints_new_triang;
        Ci_landmarks_prev = Cj_landmarks_new;
        kp_tracks = updated_kp_tracks;

        fprintf('\n');
    end
    fprintf('...VO-pipeline terminated.\n');

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
end

% display full map and cameras
if params.show_map_and_cams
    figure('name','Map landmarks');
    plotLandmarks(W_landmarks_map,'z','up');
    hold on;
    plotCam(T_WCj_vo(:,:,1),2,'black');
    plotCam(T_WCj_vo(:,:,2:end),2,'red');
end
