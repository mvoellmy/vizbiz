clear;
close all;
clc;
rng(1);

addpath(genpath('helpers'));
addpath(genpath('testing'));
addpath(genpath('visualization'));

%% Load parameter struct
fprintf('load parameter struct...\n');
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
    fprintf('loading KITTI dataset...\n');
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
    fprintf('loading MALAGA dataset...\n');
elseif params.ds == 2
    parking_path = '../datasets/parking';
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
     
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    fprintf('loading PARKING dataset...\n');
else
    assert(false);
end

%% Bootstraping
fprintf('setup boostrapping...\n');
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

T_CiCj_vo_i = NaN(4,4,numel(range_cont)+2); % transformation matrix between frame i and j, range +2 due to init
T_WCi_vo = NaN(4,4,numel(range_cont)+2); % transformation matrix between frame W and frame i range +2 due to init

%% Code profiling
if params.perf.profiling
    profile on; % trigger code profiling
end

%% Initialize VO pipeline
fprintf('initialize VO pipeline...\n');
tic;
[img_init,keypoints_init,landmarks_init,T_WC2] = initPipeline(params,img0,img1,K);
toc;

% assign first two poses
T_CiCj_vo_i(:,:,1) = eye(4); % world frame init
T_CiCj_vo_i(:,:,2) = T_WC2; % first camera pose

% transformation camera 1 to world (-90° x-axis rotation)
T_WC1 = [1      0           0         0; 
         0 cos(-pi/2)   -sin(pi/2)    0;
         0 sin(-pi/2)    cos(pi/2)    0;
                 zeros(1,3)           1];
    
% update stacked world reference pose
T_WCi_vo(:,:,1) = T_WC1* T_CiCj_vo_i(:,:,1);
T_WCi_vo(:,:,2) = T_WCi_vo(:,:,1)* T_CiCj_vo_i(:,:,2);

fprintf('...initialization done.\n\n');

%% Continuous operation VO pipeline

if params.run_continous
    
fprintf('start continuous VO operation...\n');

global fig_cont;
fig_cont = figure('name','Contiunous VO estimation');

prev_img = img_init;
keypoints_prev = keypoints_init;
landmarks_prev = landmarks_init;

for i = range_cont
    frame_idx = i-bootstrap_frame_idx_2+2; % due to init +2
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if params.ds == 0
        img = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
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
    tic;
    [T_CiCj_vo_i(:,:,frame_idx),keypoints_new,landmarks_new] = processFrame(params,img,prev_img,keypoints_prev,landmarks_prev,K);
    toc;
    
    % append newest position and rotation to logging variables
    T_WCi_vo(:,:,frame_idx) = T_WCi_vo(:,:,frame_idx-1)*T_CiCj_vo_i(:,:,frame_idx);

    % enable plots to refresh
    pause(0.01);

    % update previous image, keypoints and landmarks
    %prev_img = img;
    %keypoints_prev = keypoints_new;
    %landmarks_prev = landmarks_new;
    
    fprintf('\n\n');
end
fprintf('...VO-pipeline terminated.\n');

%% Accuracy/Precision summary
fprintf('display results...\n');
if (params.ds~=1 && params.compare_against_groundthruth)
    % plot VO trajectory against ground truth   
    plotGroundThruth_2D(T_WCi_vo(1:3,4,:),ground_truth');    
end

end

%% Profiling
if params.perf.profiling
    profile viewer; % view profiling results
end


