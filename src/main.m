clear;
close all;
clc;
rng(1);

addpath(genpath('helpers'));
addpath(genpath('testing'));
addpath(genpath('visualization'));

%% Load parameter struct
fprintf('load parameter struct...\n');
mode = 1; % todo: 1= normal, 2= ...
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
    last_frame = 598;
    K = load([parking_path '/K.txt']);
     
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    fprintf('load PARKING dataset...\n');
else
    assert(false);
end

%% Bootstraping
fprintf('setup boostrapping...\n\n');
% set bootstrap_frames % todo move into bootstrapFrames() completely??
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
fprintf('\ninitialize VO pipeline...\n');
tic;
[img_init,keypoints_init,C1_landmarks_init,T_C1C2] = initPipeline(params,img0,img1,K);
toc;

% assign first two poses
T_CiCj_vo_j(:,:,1) = eye(4); % world frame init, C1 to W
T_CiCj_vo_j(:,:,2) = T_C1C2; % first camera pose, C2 to C1

% transformation C1 to world (90deg x-axis rotation) % todo: use zeros instead?
T_WC1 = [1      0           0        0;
         0 cos(-pi/2)   -sin(-pi/2)    0;
         0 sin(-pi/2)    cos(-pi/2)    0;
                 zeros(1,3)          1];

% update stacked world-referenced pose
T_WCj_vo(:,:,1) = T_WC1; %T_WC1* T_CiCj_vo_i(:,:,1); % C1 to W
T_WCj_vo(:,:,2) = T_WC1*T_C1C2; %T_WCi_vo(:,:,1)* T_CiCj_vo_i(:,:,2); % C2 to W

% transform init point cloud to world frame
W_P_hom_init = T_WC1*[C1_landmarks_init; zeros(1,size(C1_landmarks_init,2))];
W_landmarks_init = W_P_hom_init(1:3,:);

% display initialization landmarks and bootstrap motion
if params.init.show_landmarks
    figure('name','Landmarks and motion of bootstrap image pair');
    hold on;
    plotLandmarks(W_landmarks_init);
    plotCam(T_WCj_vo(:,:,1),2,'black');
    plotCam(T_WCj_vo(:,:,2),2,'red');
end

fprintf('...initialization done.\n\n');

%% Continuous operation VO pipeline
fprintf('start continuous VO operation...');

global fig_cont fig_RANSAC_debug;
fig_cont = figure('name','Contiunous VO estimation');
fig_RANSAC_debug = figure('name','p3p / DLT estimation RANSAC');

% hand-over initialization variables
img_prev = img_init;
keypoints_prev = keypoints_init;
Ci_landmarks_prev = T_C1C2(1:3,1:3)'*C1_landmarks_init; % express in C2

for j = range_cont
    frame_idx = j-bootstrap_frame_idx_2+2; % due to init +2
    fprintf('\n\nProcessing frame %d\n=====================\n', j);
    if params.ds == 0 % todo move into currentFrames() ??
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

    if size(keypoints_prev,2) > 0 % todo: minimum number?
        tic;
        % process newest image
        [T_CiCj_vo_j(:,:,frame_idx),keypoints_new,Cj_landmarks_new] = processFrame(params,img,img_prev,keypoints_prev,Ci_landmarks_prev,K);
        toc;
    else
        warning('No keypoints left!!');
        break;
    end
    
    % append newest Ci to T transformation
    T_WCj_vo(:,:,frame_idx) = T_WCj_vo(:,:,frame_idx-1)*T_CiCj_vo_j(:,:,frame_idx);

    % enable plots to refresh
    pause(1.01);

    % update previous image, keypoints and landmarks
    img_prev = img;
    keypoints_prev = keypoints_new;
    Ci_landmarks_prev = Cj_landmarks_new;
    
    fprintf('\n\n');
end
fprintf('...VO-pipeline terminated.\n');

if params.perf.profiling
    profile viewer; % view profiling results
end

%% Performance summary
fprintf('display results...\n');
if (params.ds~=1 && params.compare_against_groundthruth)
    % plot VO trajectory against ground truth   
    plotGroundThruth_2D(T_WCj_vo(1:3,4,:),ground_truth');    
end
