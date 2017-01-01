function runVOPipeline(params, handles)
% Returns collection of parameters used throughout the VO pipeline.
% 
% Input: todo
%  - params(struct) : parameter struct
%  - handles(handles) : GUI handles
%
% Output: none

if nargin < 2
    params.through_gui = false;
end

global gui_handles;
if params.through_gui
    gui_handles = handles;
end

%% Setup datasets
updateConsole(params, 'loading parameters...\n');

if params.ds == 0
    params.kitti_path = '../datasets/kitti';
    assert(isfield(params, 'kitti_path') ~= 0);    
    ground_truth = load([params.kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
    updateConsole(params, 'load KITTI dataset...\n');
elseif params.ds == 1
    params.malaga_path = '../datasets/malaga-urban-dataset-extract-07';
    assert(isfield(params, 'malaga_path') ~= 0);
    last_frame = 2121;
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
    updateConsole(params, 'load MALAGA dataset...\n');
elseif params.ds == 2
    params.parking_path = '../datasets/parking';
    assert(isfield(params, 'parking_path') ~= 0);
    ground_truth = load([params.parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 598;
    K = load([params.parking_path '/K.txt']);
    updateConsole(params, 'load PARKING dataset...\n');
else
    assert(false);
end

pause(1);

%% Bootstraping
updateConsole(params, 'setup boostrapping...\n');

tic;

% set bootstrap frames
[img0, img1, ~, bootstrap_frame_idx_2] = autoBootstrap(params, K);
deltaT = toc;

% update gui metrics
if params.through_gui
    gui_updateMetrics(params, deltaT, gui_handles.text_RT_value);
end

updateConsole(params, '...boostrapping done.\n');

%% Setup logging variables
% set range of images to run on
if (params.cont.run_on_first_x_images > 0)
    range_cont = (bootstrap_frame_idx_2+1):(bootstrap_frame_idx_2+...
             params.cont.run_on_first_x_images);
else
    range_cont = (bootstrap_frame_idx_2+1):last_frame;
end

T_CiCj_vo_j = NaN(4,4,numel(range_cont)+2); % transformation matrix from frame Cj to Ci, range +2 due to init
T_WCj_vo = NaN(4,4,numel(range_cont)+2); % transformation matrix from frame Cj to W, range +2 due to init

%% Code profiling
if params.perf.profiling
    profile on; % trigger code profiling
end

%% Initialize VO pipeline
updateConsole(params, 'initialize VO pipeline...\n');

tic;

% transformation C1 to W (90deg x-axis rotation)
T_WC1 = [1      0           0       0;
         0      0           1       0;
         0     -1           0       0;
                 zeros(1,3)         1];

% initialize pipeline with bootstrap images
[img_init, keypoints_init, C2_landmarks_init, T_C1C2] = initPipeline(params,img0,img1,K, T_WC1);

% assign first two poses
T_CiCj_vo_j(:,:,1) = eye(4); % world frame, C1 to C1
T_CiCj_vo_j(:,:,2) = T_C1C2; % first camera pose, C2 to C1

% update stacked world-referenced pose
T_WCj_vo(:,:,1) = T_WC1; % C1 to W
T_WC2 = T_WC1*T_C1C2;
T_WCj_vo(:,:,2) = T_WC2; % C2 to W

% 2D trajectory in world frame
W_traj = T_WCj_vo(1:2,4,1);
W_traj(:,2) = T_WCj_vo(1:2,4,2);

% transform init point cloud to world frame
W_P_hom_init = T_WC1*T_C1C2*[C2_landmarks_init; ones(1,size(C2_landmarks_init,2))];
W_landmarks_init = W_P_hom_init(1:3,:);

% full 3D map point cloud in frame W
W_landmarks_map = W_landmarks_init;

deltaT = toc;

if params.through_gui
    % update gui metrics
    gui_updateMetrics(params, deltaT, gui_handles.text_RT_value);
    
    % update gui trajectory
    gui_updateTrajectory(W_traj, gui_handles.ax_trajectory, gui_handles.plot_trajectory);

    % update gui local cloud
    gui_updateLocalCloud(W_landmarks_init, gui_handles.ax_trajectory, gui_handles.plot_local_cloud);
end

% display initialization landmarks and bootstrap motion
if params.init.show_landmarks
    figure('name','Landmarks and motion of bootstrap image pair');
    hold on;
    plotLandmarks(W_landmarks_init, 'z', 'up');
    plotCam(T_WCj_vo(:,:,1), 1, 'black');
    plotCam(T_WCj_vo(:,:,2), 1, 'red');
end

updateConsole(params, '...initialization done.\n');

%% Continuous operation VO pipeline
global fig_cont fig_RANSAC_debug;

if params.run_continous
    updateConsole(params, 'start continuous VO operation...\n');
    
	% setup figure handles
	fig_cont = figure('name','Contiunous VO estimation');
	fig_RANSAC_debug = figure('name','p3p / DLT estimation RANSAC');
    
	% hand-over initialization variables
	img_prev = img_init;
    state_prev.keypoints = keypoints_init;          % keypoints found in prev state
    state_prev.Ci_landmarks = C2_landmarks_init;    % associated with landmarks
    state_prev.T_WCi = T_WC2;                       % frame pose in W

    for j = range_cont
        updateConsole(params, ['Processing frame ',num2str(j),'\n']);
        
        tic;
        
        % pick current frame
        frame_idx = j - bootstrap_frame_idx_2 + 2; % due to init +2
        img = currentFrame(params, j);
        
        if (size(state_prev.keypoints,2) > 10) % todo: minimum number?          
            [T_CiCj_vo_j(:,:,frame_idx), state] = processFrame(params, img, img_prev, state_prev, K);
            
            % add super title with frame number
            if params.cont.show_current_image
                figure(fig_cont);
                suptitle(sprintf('Frame #%i',j));
            end
        else
            warning('Too few keypoints left!! Break continuous operation loop - Terminating...');
            break;
        end

        % append newest Cj to W transformation
        T_WCj_vo(:,:,frame_idx) = T_WCj_vo(:,:,frame_idx-1)*T_CiCj_vo_j(:,:,frame_idx);

        % extend 2D trajectory
        W_traj =[W_traj T_WCj_vo(1:2,4,frame_idx)];
        
        % update map with new landmarks
        W_P_hom_new = T_WCj_vo(:,:,frame_idx)*[Cj_landmarks_new; ones(1, size(Cj_landmarks_new,2))];
        W_landmarks_new = W_P_hom_new(1:3,:);
        W_landmarks_map = [W_landmarks_map, W_landmarks_new];
        
        if params.through_gui
            % update gui trajectory
            gui_updateTrajectory(W_traj, gui_handles.ax_trajectory, gui_handles.plot_trajectory);
            % update gui local cloud
            gui_updateLocalCloud(W_landmarks_new, gui_handles.ax_trajectory, gui_handles.plot_local_cloud);
        end

        % allow plots to refresh
        pause(1.01);       

        % update previous image and state(keypoints, landmarks, pose)
        img_prev = img;
        state_prev.keypoints = state.keypoints;
        state_prev.Ci_landmarks = state.Cj_landmarks;
        state_prev.T_WCi = state.T_WCj;

        updateConsole(params, ' \n');
        
        deltaT = toc;
        
        if params.through_gui
            % update gui metrics
            gui_updateMetrics(params, deltaT, gui_handles.text_RT_value);
        end
    end
    updateConsole(params, '...VO-pipeline terminated.\n');
end

%% Results summary
updateConsole(params, 'display results...\n');

if (params.ds ~= 1 && params.compare_against_groundthruth)
    % plot VO trajectory against ground truth   
    plotTrajectoryVsGT_2D(T_WCj_vo(1:3,4,:),ground_truth');
elseif (params.ds == 1 && params.compare_against_groundthruth)
    % plot VO trajectory
    plotTrajectory_2D(T_WCj_vo(1:3,4,:));
end

% display full map and cameras
if params.show_map_and_cams
    figure('name', 'Map landmarks');
    plotLandmarks(W_landmarks_map, 'z', 'up');
    hold on;
    plotCam(T_WCj_vo(:,:,1), 2, 'black');
    plotCam(T_WCj_vo(:,:,2:end), 2, 'red');
end

end
