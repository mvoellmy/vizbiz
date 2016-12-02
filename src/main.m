clear;
close all;
clc;

addpath(genpath('helpers'));
addpath(genpath('visualization'));

%% Load parameter struct
disp('loading parameter struct...');
mode = 1;
p = loadParameters(mode);

%% Setup
ds = 0; % 0: KITTI, 1: Malaga, 2: parking

if ds == 0
    kitti_path = '../datasets/kitti';
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
    disp('loading KITTI dataset...');
elseif ds == 1
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
elseif ds == 2
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
if ds == 0
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrapFrames(ds,1))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrapFrames(ds,2))]);
elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrapFrames(ds,1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrapFrames(ds,2)).name]));
elseif ds == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrapFrames(ds,1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrapFrames(ds,2))]));
else
    assert(false);
end

% display boostrap images
if p.show_bootstrap_images    
    figure('name','Boostrap images');
    subplot(1,2,1);
    imshow(img0);
    subplot(1,2,2);
    imshow(img1);
end

if p.perf.profiling
    % trigger code profiling
    profile on;
end

%% Initialize VO pipeline
disp('initialize VO pipeline...');
tic;
[img_init, keypoints_init, landmarks_init] = initPipeline(p, img0, img1,K);
toc;
disp('...initialization done.');

%% Continuous operation VO pipeline
disp('start continuous VO operation...');
fig1 = figure('name','Contiunous VO estimation');

% set range of images to run on
if p.cont.run_on_first_ten_images
    range = (bootstrapFrames(ds,2)+1):(bootstrapFrames(ds,2)+10); % +1 due to init
else
    range = (bootstrapFrames(ds,2)+1):last_frame;
end

% Logging variables
vo_t_WC_i = NaN(3, numel(range)); % delta translation in last camera frame
vo_R_WC_i = NaN(3,3, numel(range)); % Rotation matrix between frame i and i-1
W_p_C = NaN(3, numel(range)); % Absolute position of camera in world frame
prev_img = img_init;
keypoints_prev = keypoints_init;
landmarks_map = landmarks_init(1:3,:);

for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        img = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
        imshow(img);
    elseif ds == 1
        img = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        img = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end

    % process newest image
    [R_WC_i, t_WC_i, keypoints_new, landmarks_map] = ...
        processFrame(img,prev_img,keypoints_prev,landmarks_map,K,fig1);
    
    % append newest position and rotation to logging variables
    vo_t_WC_i(:, i-range(1)+1) = t_WC_i;
    vo_R_WC_i(:, :, i-range(1)+1) = R_WC_i;
    
    if (i==range(1)) % first init
        W_p_C(:, i-range(1)+1) = [0;0;0];
    else
        W_p_C(:, i-range(1)+1) = W_p_C(i-1)+(R_WC_i*t_WC_i);
    end
    
    % enable plots to refresh
    pause(1.01);

    % update previous image and keypoints
    % (enable once keypoint tracks defined)
    % prev_img = img;
    % keypoints_prev = keypoints_new;
end
disp('...VO-pipeline terminated.');
    
if p.perf.profiling
    % view profiling results
    profile viewer;
end

%% Performance summary
disp('display results...');
if (ds~=1 && p.compare_against_groundthruth)
    % plot VO trajectory against ground truth   
    
    figure('name','Comparison against ground truth');
    hold on;
    plot(ground_truth(:,1),ground_truth(:,2),'k-');
    plot(W_p_C(1,:),W_p_C(2,:),'*', 'MarkerSize',20);
    plot(ground_truth(1,1),ground_truth(1,2),'ksquare');
    plot(ground_truth(end,1),ground_truth(end,2),'ko');

    xlabel('x');
    ylabel('y');
    legend('ground truth','visual odometry','start','end');
end


