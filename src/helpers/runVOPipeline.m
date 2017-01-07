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
    ground_truth = load([params.malaga_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
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
elseif params.ds == 3
    params.polyUp_path = '../datasets/poly_up';
    assert(isfield(params, 'polyUp_path') ~= 0);
    ground_truth = load([params.polyUp_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 602;
    K = [1851.1 0   560.1;
         0   1848.4  924.1;
         0   0         1];
    updateConsole(params, 'load POLY-UP dataset...\n');
elseif params.ds == 4
    params.polyDown_path = '../datasets/poly_down';
    assert(isfield(params, 'polyDown_path') ~= 0);
    ground_truth = load([params.polyDown_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 599;
    K = [1851.1 0   560.1;
         0   1848.4  924.1;
         0   0         1];
    updateConsole(params, 'load POLY-DOWN dataset...\n');
else
    assert(false);
end

pause(1);

%% Bootstraping
updateConsole(params, 'setup boostrapping...\n');

% set bootstrap frames
[img0, img1, bootstrap_frame_idx_1, bootstrap_frame_idx_2] = getBootstrapFrames(params, K);

updateConsole(params, '...boostrapping done.\n\n');

%% Setup logging variables
% set range of images to run on
if (params.run_on_first_x_images > 0)
    range_cont = (bootstrap_frame_idx_2+1):(bootstrap_frame_idx_2 + params.run_on_first_x_images);
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

global fig_kp_tracks;
if params.cont.figures && params.kp_tracker.figures
    fig_kp_tracks = figure('name','Keypoint tracker');
end

tic;

% transformation C1 to W (90deg x-axis rotation)
T_WC1 = [1      0           0       0;
         0      0           1       0;
         0     -1           0       0;
                 zeros(1,3)         1];

% initialize pipeline with bootstrap images
[img_init, keypoints_first_frame, keypoints_second_frame, C2_landmarks_init, T_C1C2, kp_tracks, norm_scale] = ...
    initPipeline(params, img0, img1, K, T_WC1, 1, ground_truth, bootstrap_frame_idx_1, bootstrap_frame_idx_2);


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
    gui_updateMetrics(deltaT, gui_handles.text_RT_value);
    
    % update tracking metric
    gui_updateTracked(params, size(keypoints_second_frame,2),...
                      gui_handles.text_value_tracked, gui_handles.ax_tracked, gui_handles.plot_bar);

    % update ground truth
    gui_updateGT(ground_truth, gui_handles.ax_trajectory, gui_handles.plot_gt);
    
    % update gui local cloud
    gui_updateLocalCloud(W_landmarks_init, gui_handles.ax_trajectory, gui_handles.plot_local_cloud);
    
    % update gui trajectory
    gui_updateTrajectory(W_traj, gui_handles.ax_trajectory, gui_handles.plot_trajectory);    
end

% display initialization landmarks and bootstrap motion
if (params.init.figures && params.init.show_landmarks)
    figure('name','Landmarks and motion of initialization image pair');
    hold on;
    plotLandmarks(W_landmarks_init, 'z', 'up');
    plotCam(T_WCj_vo(:,:,1), 0.2, 'black');
    plotCam(T_WCj_vo(:,:,2), 0.2, 'red');
end

updateConsole(params, '...initialization done.\n\n');

%% Continuous operation VO pipeline
global fig_cont fig_RANSAC_debug fig_kp_triangulate;

if params.run_continous
    updateConsole(params, 'start continuous VO operation...\n');
    
	% setup figure handles
    if params.cont.figures
        fig_cont = figure('name','Contiunous VO estimation');
        if params.localization_ransac.show_iterations
            fig_RANSAC_debug = figure('name','p3p / DLT estimation RANSAC');
        end
        if params.kp_tracker.figures
            fig_kp_triangulate = figure('name', 'triangulate');   
        end
        fig_debug_traj = figure('name','Trajectory');
    end
    
	% hand-over initialization variables
	img_prev = img_init;
    keypoints_prev_triang = keypoints_second_frame;
    Ci_landmarks_prev = C2_landmarks_init;    

    % fill first two frames to bundleAdjust container
    if params.cont.use_BA
        ba_keypoints_init = [flipud(keypoints_first_frame); flipud(keypoints_second_frame)];
        ba_view_ids = [1, 2];   % View ids for init
        for i=1:size(keypoints_second_frame, 2) % todo: pretty sure this can be indexed
            ba_p_corresponding = vec2mat(ba_keypoints_init(:,i),2);
            ba_point_tracks(i) = pointTrack(ba_view_ids, ba_p_corresponding);
        end

        ba_orientations = cell(2,1);
        ba_locations = cell(2,1);
        ba_orientations(1) = {T_WC1(1:3, 1:3)'}; % This is transposed as we want the orientation of the cam in the world frame
        ba_locations(1) = {T_WC1(1:3, 4)'};
        ba_orientations(2) = {T_WC2(1:3, 1:3)'};
        ba_locations(2) = {T_WC2(1:3, 4)'};
        
        frames_since_ba = -1; % this variable keeps count of when we last bundle adjusted. -1 means we should do it the next time we can.
        ba_fixed_view_ids = 1;
    end
    
    for img_idx = range_cont
        updateConsole(params, ['Processing frame ',num2str(img_idx),'\n']);
        
        tic;
        
        % pick current frame, due to initialization +2
        frame_idx = img_idx - bootstrap_frame_idx_2 + 2;
        img_new = getFrame(params, img_idx);
        
        if (size(keypoints_prev_triang,2) > 6 && size(Ci_landmarks_prev, 2) > 6) % todo: minimum number?
            % Continuity check
            if (size(keypoints_prev_triang, 2) ~= size(Ci_landmarks_prev, 2))
                updateConsole(params, 'keypoints and landmarks have different dimensions!! Break continuous operation loop - Terminating...');
                break;
            end
            
            % extract current camera pose
            T_WCi = T_WCj_vo(:,:,frame_idx-1);
            
            % choose img for reinit
            reInitFrameNr = max( [1, frame_idx - params.cont.reinit.deltaFrames] );
            img_reInit = getFrame(params, reInitFrameNr);
            
            % create bootstrap idx
            bootstepIdx.first = reInitFrameNr;
            bootstepIdx.second = frame_idx;
            
            % save Pose for reInit
            T_WCinit = T_WCj_vo(:,:,reInitFrameNr);
                        
            % process newest image
            [T_CiCj, keypoints_new_triang, updated_kp_tracks, Cj_landmarks_j, p_candidates_first_inliers, p_candidates_j_inliers_nr_tracking, reInitFlag] =...
                processFrame(params, img_new, img_prev, img_reInit, T_WCinit, keypoints_prev_triang, kp_tracks, Ci_landmarks_prev, T_WCi, K, norm_scale);

            
            % check if reInit was performed
            if (reInitFlag)
                
                % check if reinitialization before enough steps performed
                assert ((reInitFrameNr - 1) > 0);
                T_WCj_vo(:,:,reInitFrameNr:frame_idx) = T_WCj_vo(:,:,reInitFrameNr - 1);
                T_CiCj_vo_j(:,:,reInitFrameNr:frame_idx) = eye(4);                
                
                % set parameter, such that the frame will be bundleadjusted
                frames_since_ba = -1;
            end
            % append last pose
            T_CiCj_vo_j(:,:,frame_idx) = T_CiCj;
                        
            % add super title with frame number
            if params.cont.figures
                figure(fig_cont);
                %suptitle(sprintf('Frame #%i',j));
            end
        else
            updateConsole(params, 'Too few keypoints left!! Break continuous operation loop - Terminating...');
            break;
        end
        if size(Cj_landmarks_j, 2) == 0
            updateConsole(params, 'No landmarks left!! Break continuous operation loop - Terminating...');
            break;
        end
                  
        % append newest Cj to W transformation
        T_WCj_vo(:,:,frame_idx) = T_WCj_vo(:,:,frame_idx-1) * T_CiCj_vo_j(:,:,frame_idx);
    
        W_P_hom_j = T_WCj_vo(:,:,frame_idx) * [Cj_landmarks_j; ones(1, size(Cj_landmarks_j, 2))];

        % Add keypoints to corresponding landmarks
        nr_matched_landmarks = size(W_P_hom_j, 2);
        nr_new_landmarks = size(p_candidates_j_inliers_nr_tracking, 2);
        nr_old_landmarks = nr_matched_landmarks - nr_new_landmarks;
              
        if params.cont.use_BA
            % todo: yeah it's not very efficient.
            % Ideally we would be storing the indeces of the landmarks
            % which were mapped, then would would have to do 0 search.
            % However I don't think we have time/the framework for that atm.

            % matching to ba_tracking
            tolerance = 10^-6;
            missed_landmarks_count = 0;
            % indices of the current frame landmarks within the map, used
            % to find the current frame landmarks after BA.
            idx_of_matched_in_map = [];
            
            % apply spherical filter on map to search through for matching
            Cj_P_hom_map = tf2invtf(T_WCj_vo(:,:,frame_idx)) * [W_landmarks_map; ones(1, size(W_landmarks_map, 2))];
            [~, outFOV_idx] = applySphericalFilter(params, Cj_P_hom_map, 2*params.cont.landmarks_cutoff);
            Cj_P_hom_visible = Cj_P_hom_map;
            Cj_P_hom_visible(:,outFOV_idx) = zeros(4,nnz(outFOV_idx));
            updateConsole(params, sprintf('Searching through %i map landmarks \n', nnz(Cj_P_hom_visible(1,:))));
            
            for it=1:nr_old_landmarks
                % Find corresponding Landmark indices
                ba_index = find(abs(Cj_P_hom_visible(1,:) - Cj_landmarks_j(1,it)) < tolerance & ...
                                abs(Cj_P_hom_visible(2,:) - Cj_landmarks_j(2,it)) < tolerance & ...
                                abs(Cj_P_hom_visible(3,:) - Cj_landmarks_j(3,it)) < tolerance);

                if nnz(ba_index) > 0
                    if size(ba_index, 2) > 1
                        % if landmark correspond to multiple landmarks on
                        % the map. Match it to the closest one.
                        error = sqrt((Cj_P_hom_visible(1,ba_index(1:size(ba_index, 2))) - Cj_landmarks_j(1,it)).^2 + ...
                                     (Cj_P_hom_visible(2,ba_index(1:size(ba_index, 2))) - Cj_landmarks_j(2,it)).^2 + ...
                                     (Cj_P_hom_visible(3,ba_index(1:size(ba_index, 2))) - Cj_landmarks_j(3,it)).^2);
                        [~, min_idx] = min(error);
                        ba_index = ba_index(min_idx);
                    end
                    
                    ba_point_tracks(ba_index).Points = [ba_point_tracks(ba_index).Points; keypoints_new_triang(2, it),  keypoints_new_triang(1, it)];
                    ba_point_tracks(ba_index).ViewIds = [ba_point_tracks(ba_index).ViewIds, frame_idx];
                    idx_of_matched_in_map = [idx_of_matched_in_map, ba_index];
                else
                    missed_landmarks_count = missed_landmarks_count+1;
                end
            end
        updateConsole(params, sprintf(' ---->%i landmarks where not matched again!\n', missed_landmarks_count));
% for debugging purposes
            
            % add new landmarks to tracking
            for it = 1:nr_new_landmarks
                ba_point_tracks(size(ba_point_tracks, 2) + 1) = pointTrack([frame_idx-p_candidates_j_inliers_nr_tracking(it), frame_idx],...
                    [p_candidates_first_inliers(2, it)           , p_candidates_first_inliers(1, it);...
                     keypoints_new_triang(2, nr_old_landmarks+it), keypoints_new_triang(1, nr_old_landmarks+it)]);
            end
            
            map_size = size(W_landmarks_map, 2);
            idx_of_matched_in_map = [idx_of_matched_in_map, map_size + 1:map_size + nr_new_landmarks];       
        end
        
        % update map
        W_landmarks_map = [W_landmarks_map, W_P_hom_j(1:3, nr_old_landmarks+1:end)];
 
        if params.cont.use_BA  
            % fill BA-container with poses
            ba_orientations(frame_idx) = {T_WCj_vo(1:3, 1:3, frame_idx)'};
            ba_locations(frame_idx) = {T_WCj_vo(1:3, 4, frame_idx)'};

            ba_view_ids = [ba_view_ids, frame_idx];
            cameraPoses = table;
            cameraPoses.ViewId = uint32(ba_view_ids');
            cameraPoses.Orientation = ba_orientations;
            cameraPoses.Location = ba_locations;
            cameraParams = cameraParameters('IntrinsicMatrix', K');
            
            if frames_since_ba >= params.cont.ba.frequency || frames_since_ba == -1
                [W_landmarks_map, refinedPoses] = bundleAdjustment(W_landmarks_map', ba_point_tracks, cameraPoses, cameraParams, 'FixedViewIDs', ba_fixed_view_ids);
                refined_frames_string = sprintf('%d ', refinedPoses.ViewId);
                updateConsole(params, sprintf(' bundle-adjusted poses and landmarks in frames %s \n', refined_frames_string));
                W_landmarks_map = W_landmarks_map'; % transposed because of MATLAB function interface/output

                % append to trajectory
                T_WCj_vo(1:3,1:3,refinedPoses.ViewId') = reshape(cell2mat(refinedPoses.Orientation(refinedPoses.ViewId'))',3,3,length(refinedPoses.ViewId));
                T_WCj_vo(1:3,4,refinedPoses.ViewId') = reshape(cell2mat(refinedPoses.Location(refinedPoses.ViewId'))',1,3,length(refinedPoses.ViewId));
                T_WCj_vo(4,1:3,refinedPoses.ViewId') = zeros(1,3,length(refinedPoses.ViewId));
                T_WCj_vo(4,4,refinedPoses.ViewId') = ones(1,1,length(refinedPoses.ViewId));
                
                % update landmarks current frame
                W_landmarks_last_frame = W_landmarks_map(:, idx_of_matched_in_map);            
                Cj_P_hom_j = tf2invtf(T_WCj_vo(1:4, 1:4, frame_idx)) * [W_landmarks_last_frame; ones(1,size(W_landmarks_last_frame,2))];
                Cj_landmarks_j = Cj_P_hom_j(1:3,:);
                frames_since_ba = 0;
                
                % fix bundleadjusted poses
                if params.cont.ba.fix_view_ids
                    ba_fixed_view_ids = refinedPoses.ViewId';
                end
            end
            frames_since_ba = frames_since_ba + 1;
        end
        
        % extend 2D trajectory
        W_traj =[W_traj, T_WCj_vo(1:2,4,frame_idx)];
        if params.cont.figures
            figure(fig_debug_traj);
            plotGroundThruth_3D(squeeze(T_WCj_vo(1:3,end,1:frame_idx)), ground_truth);
        end

        if params.through_gui
            % update tracking metric
            gui_updateTracked(params, size(keypoints_new_triang,2),...
                              gui_handles.text_value_tracked, gui_handles.ax_tracked, gui_handles.plot_bar);
            % update gui local cloud
            gui_updateLocalCloud(W_P_hom_j, gui_handles.ax_trajectory, gui_handles.plot_local_cloud);
            % update gui trajectory
            gui_updateTrajectory(W_traj, gui_handles.ax_trajectory, gui_handles.plot_trajectory);            
        end

        % allow plots to refresh
        pause(0.01);

        % update previous image, keypoints, landmarks and tracker
        img_prev = img_new;
        keypoints_prev_triang = keypoints_new_triang;
        Ci_landmarks_prev = Cj_landmarks_j;
        kp_tracks = updated_kp_tracks;

        updateConsole(params, ' \n');
        
        deltaT = toc;
        
        if params.through_gui
            % update gui metrics
            gui_updateMetrics(deltaT, gui_handles.text_RT_value);
        end
    end % end for loop
    updateConsole(params, '...VO-pipeline terminated.\n');
end

%% Results summary
if (params.ds ~= 1 && params.compare_against_groundthruth)
    % plot VO trajectory against ground truth   
    plotTrajectoryVsGT_2D(T_WCj_vo(1:3,4,:),ground_truth', bootstrap_frame_idx_1, bootstrap_frame_idx_2);
elseif (params.ds == 1 && params.compare_against_groundthruth)
    % plot VO trajectory
    plotTrajectory_2D(T_WCj_vo(1:3,4,:), bootstrap_frame_idx_1, bootstrap_frame_idx_2);
end

% display full map and cameras
if params.show_map_and_cams
    figure('name', 'Map landmarks');
    plotLandmarks(W_landmarks_map, 'z', 'up');
    hold on;
    plotCam(T_WCj_vo(:,:,1), 0.2, 'black');
    plotCam(T_WCj_vo(:,:,2:end), 0.2, 'red');
end
end