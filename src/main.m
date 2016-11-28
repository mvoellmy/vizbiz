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
[I_init, keypoints_init, landmarks_init] = initVOpipeline(p, img0, img1);
toc;
disp('...initialization done.');

%% Continuous operation
disp('start continuous VO operation...');
fig1 = figure('name','Contiunous VO estimation');

if p.cont.run_on_first_ten_images
    range = (bootstrapFrames(ds,2)+1):(bootstrapFrames(ds,2)+10);
else
    range = (bootstrapFrames(ds,2)+1):last_frame;
end

for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end

    % process newest image
    processFrame(image,fig1);

    % enable plots to refresh
    pause(0.01);

    % update previous image
    prev_img = image;
end
disp('...VO-pipeline terminated.');
    
if p.perf.profiling
    % view profiling results
    profile viewer;
end
