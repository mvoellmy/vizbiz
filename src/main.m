clear;
close all;
clc;
rng(1);

addpath(genpath('helpers'));
addpath(genpath('visualization'));

%% Load parameter struct
disp('load parameter struct...');
mode = 1; % 1: normal, 2: ...
params = loadParameters(mode);

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
    disp('loading KITTI dataset...');
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
    disp('loading MALAGA dataset...');
elseif params.ds == 2
    parking_path = '../datasets/parking';
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
     
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    disp('loading PARKING dataset...');
else
    assert(false);
end

%% Bootstraping
disp('setup boostrapping...');
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
if params.show_bootstrap_images    
    figure('name','Boostrap images');
    subplot(1,2,1);
    imshow(img0);
    subplot(1,2,2);
    imshow(img1);
end

%% Code profiling
if params.perf.profiling    
    profile on; % trigger code profiling
end

%% Initialize VO pipeline
disp('initialize VO pipeline...');
tic;
[img_init,keypoints_init,landmarks_init] = initPipeline(params,img0,img1,K);
toc;
disp('...initialization done.');
return;
%% Continuous operation VO pipeline
disp('start continuous VO operation...');
fig1 = figure('name','Contiunous VO estimation');

% set range of images to run on
if params.cont.run_on_first_ten_images
    range = (bootstrapFrames(params.ds,'second')+1):(bootstrapFrames(params.ds,'second')+10); % +1 due to init
else
    range = (bootstrapFrames(params.ds,'second')+1):last_frame;
end

% logging variables
W_vo_t_WC_i = NaN(3,numel(range)); % delta translation in last camera frame
W_vo_R_WC_i = NaN(3,3,numel(range)); % rotation matrix between frame i and i-1
W_Pos_C = NaN(3,numel(range)); % absolute position of camera in world frame
prev_img = img_init;
keypoints_prev = keypoints_init;
landmarks_map = landmarks_init(1:3,:);

for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if params.ds == 0
        img = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
        imshow(img);
    elseif params.ds == 1
        img = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif params.ds == 2
        img = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end

    % process newest image
    [R_WC_i,t_WC_i,keypoints_new, landmarks_map] = ...
        processFrame(img,prev_img,keypoints_prev,landmarks_map,K,fig1);
    
    % append newest position and rotation to logging variables
    W_vo_t_WC_i(:,i-range(1)+1) = t_WC_i;
    W_vo_R_WC_i(:,:,i-range(1)+1) = R_WC_i;
    
    if (i==range(1)) % first init
        W_Pos_C(:,i-range(1)+1) = [0;0;0];
    else
        W_Pos_C(:,i-range(1)+1) = W_Pos_C(i-1)+(R_WC_i*t_WC_i);
    end
    
    % enable plots to refresh
    pause(0.01);

    % update previous image and keypoints
    % (enable once keypoint tracks defined)
    % prev_img = img;
    % keypoints_prev = keypoints_new;
end
disp('...VO-pipeline terminated.');

if params.perf.profiling
    profile viewer; % view profiling results
end

%% Performance summary
disp('display results...');
if (params.ds~=1 && params.compare_against_groundthruth)
    % plot VO trajectory against ground truth   
    plotGroundThruth_2D (W_Pos_C,ground_truth');    
end
