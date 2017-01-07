function [I_init,keypoints_first_frame , keypoints_second_frame, C2_landmarks_init, T_C1C2, kp_tracks_init, norm_scale] =...
    initPipeline(params, I_i1, I_i2, K, T_WC1, scale_in, ground_truth, bootstrap_frame_idx_1, bootstrap_frame_idx_2)
% Returns initialization image and corresponding sorted keypoints and landmarks
% after checking for valid correspondences between a bootstrap image pair.
% Optionally, precalculated outputs are loaded.
% 
% Input:
%  - params(struct) : parameter struct
%  - I_i1(size) : first image
%  - I_i2(size) : second image
%  - K(3x3) : camera calibration matrix
%  - T_WC1(4x4) : fixed transformation from C1 to W
%  - scale_in : Precalculated scale of dataset (use 1 for first first call)
%  - ground_truth(2xM) : ground thruth positions in world frame (optional)
%  - bootstrap_frame_1_idx(1x1) : dataset image index of I_i1 (optional)
%  - bootstrap_frame_2_idx(1x1) : dataset image index of I_i2 (optional)
%
% Output:
%  - I_init(size) : initialization image
%  - keypoints_first_frame(2xN) : matched keypoints from first image , each [v,u]
%  - keypoints_second_frame(2xN) : matched keypoints from second image, each [v,u]
%  - C2_landmarks_init(3xN) : C2-referenced triangulated 3D points
%  - T_C1C2(4x4) : homogeneous transformation matrix C2 to C1
%  - kp_tracks_init(struct) : struct containing the first candidate keypoints
%  - norm_scale(1x1) : calculated scale during init

global fig_init gui_handles;

% create container for keypoint tracker (new keypoints with no landmarks)
% set of candidate keypoints in last camera frame
kp_tracks.candidate_kp = []; % 2xN
% keypoint coordinates of every candiate in its first observed frame
kp_tracks.first_obs_kp = [];  % 2xN
% keypoint pose of every candiate in its first observed frame
kp_tracks.first_obs_pose = []; % 16xN
kp_tracks.nr_trackings = []; % 1xN

if params.init.use_KITTI_precalculated_init % todo: still needed?
    % assign second image as initialization image
    I_init = I_i2;
    if params.through_gui
        gui_updateImage(I_init, gui_handles.ax_current_frame);
    end
    
    % load precalculated keypoints and landmarks
    keypoints_second_frame = load('../datasets/kitti/precalculated/keypoints.txt')';
    C2_landmarks_init = load('../datasets/kitti/precalculated/landmarks.txt')';
    
    T_C1C2 = eye(4);
    
    kp_tracks_init = kp_tracks;
else
    % assign second image as initialization image
    I_init = I_i2;
    if params.through_gui
        gui_updateImage(I_init, gui_handles.ax_current_frame);
    end
    
    % find 2D correspondences (sorted)
    [p_i1_uv, p_i2_uv, unmatched_query_kp_vu] = findCorrespondeces(params,I_i1,I_i2);

    % homogenize keypoints
    p_hom_i1_uv = [p_i1_uv; ones(1,size(p_i1_uv,2))];
    p_hom_i2_uv = [p_i2_uv; ones(1,size(p_i2_uv,2))];    

    % estimate the essential matrix E using normalized 8-point algorithm
    % and RANSAC for outlier rejection
    [E, inliers] = eightPointRansac(params, p_hom_i1_uv, p_hom_i2_uv, K, K);
    
    % extract inlier keypoints
    p_hom_inlier_i1_uv = p_hom_i1_uv(:,inliers);
    p_hom_inlier_i2_uv = p_hom_i2_uv(:,inliers);
    
    % extract the relative camera pose (R,t) from the essential matrix
    [Rots, u3] = decomposeEssentialMatrix(E);

    % disambiguate among the four possible configurations
    [R_C2C1, C2_t_C2C1] = disambiguateRelativePose(Rots, u3, p_hom_inlier_i1_uv, p_hom_inlier_i2_uv, K, K);
    
    % construct C2 to W transformation
    T_C2C1 = [R_C2C1,  C2_t_C2C1;
              zeros(1,3),      1];
    T_C1C2 = tf2invtf(T_C2C1);
    T_C1C1 = eye(4,4);    
    T_WC2 = T_WC1*T_C1C2;
    
    % triangulate a point cloud using the final transformation (R,t)
    M1 = K*T_C1C1(1:3,:);
    M2 = K*T_C2C1(1:3,:);
    C1_P_hom_init = linearTriangulation(p_hom_inlier_i1_uv, p_hom_inlier_i2_uv, M1, M2);
    W_P_hom_init = T_WC1*C1_P_hom_init;
    
    if params.init.use_BA
         if params.init.show_BA_comp && params.init.figures
            figure('name','BundleaAdjustment Comparison');
            subplot(1,2,1)
            plotLandmarks(W_P_hom_init(1:3,:),'z','up');
            hold on
            plotCam(T_WC1,20,'black');
            plotCam(T_WC2,20,'red');
            title('Before BA');
        end
        [W_P_init, T_WC_refined] = bundleAdjust(params, W_P_hom_init(1:3,:), [p_hom_inlier_i1_uv(1:2,:); p_hom_inlier_i2_uv(1:2,:)], cat(3, T_WC1, T_WC2), K, 1);
        W_P_hom_init = [W_P_init;ones(1, size(W_P_init, 2))];
        
        % update homogeneous transformations
        T_WC1 = T_WC_refined(1:4,1:4, 1);
        T_WC2 = T_WC_refined(1:4,1:4, 2);
        T_C1W = tf2invtf(T_WC1);
        T_C1C2 = T_C1W*T_WC2;
        T_C2C1 = tf2invtf(T_C1C2);
        
        C1_P_hom_init = T_C1W*W_P_hom_init;
        
        if params.init.show_BA_comp && params.init.figures
            subplot(1,2,2)
            plotLandmarks(W_P_hom_init(1:3,:),'z','up');
            hold on
            plotCam(T_WC1,20,'black');
            plotCam(T_WC2,20,'red');
            title('After BA');
        end
    end
    
    % check which scale normalization is required
    if(nargin < 7)
        % reInit mode
        % normalize scale with ground truth using precalculated scale
        if params.init.normalize_scale
            % pass fake arguments
            [C1_P_init, T_C1C2] = normalizeScale(params, C1_P_hom_init(1:3,:), T_C1C2, 1, 1, 1, scale_in);
        end
    else
        % bootstrap Init mode
        % normalize scale with ground truth by calculate scale
        if params.init.normalize_scale
            [C1_P_init, T_C1C2, norm_scale] = normalizeScale(params, C1_P_hom_init(1:3,:), T_C1C2, ground_truth, bootstrap_frame_idx_1, bootstrap_frame_idx_2);
        end
    end
        
    C1_P_hom_init = [C1_P_init; ones(1,size(C1_P_init,2))];
    
    % discard landmarks not contained in spherical neighborhood
    [C1_P_hom_init, outFOV_idx] = applySphericalFilter(params, C1_P_hom_init, params.init.landmarks_cutoff);
    % filter corresponding keypoints
    p_hom_inlier_i1_uv(:,outFOV_idx) = [];
    p_hom_inlier_i2_uv(:,outFOV_idx) = [];
    
    % assign initialization entities
    keypoints_first_frame = flipud(p_hom_inlier_i1_uv(1:2,:));
    keypoints_second_frame = flipud(p_hom_inlier_i2_uv(1:2,:));
    C2_P_hom_init = T_C2C1*C1_P_hom_init;

    C2_landmarks_init = C2_P_hom_init(1:3,:);

    % show inlier and filtered matches
    if (params.init.figures && params.init.show_keypoints && params.init.show_inlier_matches)
        figure(fig_init);
        subplot(2,2,4);
        showMatchedFeatures(I_i1, I_i2,...
                            p_hom_inlier_i1_uv(1:2,:)',...
                            p_hom_inlier_i2_uv(1:2,:)', 'blend', 'PlotOptions', {'rx','gx','y-'});
        title('Filtered inlier keypoint matches');
    end
    
    % update filtered inlier gui keypoints
    if params.through_gui && params.gui.show_inlier_features
        gui_updateKeypoints(flipud(p_hom_inlier_i2_uv(1:2,:)), gui_handles.ax_current_frame, 'g.');
    end
    
    % update gui triangulated keypoints
    if params.through_gui && params.gui.show_triang_features
        gui_updateKeypoints(flipud(p_hom_inlier_i2_uv(1:2,:)), gui_handles.ax_current_frame, 'gx');
    end
    
    % display statistics
    updateConsole(params,...
                  sprintf(['  Number of initialization keypoints: %i\n',...
                  '  Number of initialization landmarks: %i\n'],...
                  size(keypoints_second_frame,2), size(C2_landmarks_init,2)));
    
	% initialise keypoint tracker
    kp_tracks_init = updateKpTracks(params, kp_tracks,I_i1, I_i2, unmatched_query_kp_vu, T_WC2);
    
    % update gui keypoints % todo: move to initPipeline()
    if params.through_gui && params.gui.show_all_features
        gui_updateKeypoints(kp_tracks_init.candidate_kp, gui_handles.ax_current_frame, 'r.');
    end
end

% check for same number of keypoints and landmarks
assert(size(keypoints_second_frame,2) == size(C2_landmarks_init,2));
assert(size(keypoints_first_frame,2) == size(C2_landmarks_init,2));

end
