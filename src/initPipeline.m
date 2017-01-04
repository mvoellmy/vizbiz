function [I_init, keypoints_init, C2_landmarks_init, T_C1C2, kp_tracks_init] = initPipeline(params, I_i1, I_i2, K, T_WC1)
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
%
% Output:
%  - I_init(size) : initialization image
%  - keypoints_init(2xN) : matched keypoints from image pair, each [v,u]
%  - C2_landmarks_init(3xN) : C2-referenced triangulated 3D points
%  - T_C1C2(4x4) : homogeneous transformation matrix C2 to C1
%  - kp_tracks_init(struct) : TODO

global fig_init gui_handles;

if params.init.use_KITTI_precalculated_init % todo: still needed?
    % assign second image as initialization image
    I_init = I_i2;
    if params.through_gui
        gui_updateImage(I_init, gui_handles.ax_current_frame);
    end
    
    % load precalculated keypoints and landmarks
    keypoints_init = load('../datasets/kitti/precalculated/keypoints.txt')';
    C2_landmarks_init = load('../datasets/kitti/precalculated/landmarks.txt')';
    
    T_C1C2 = eye(4);
else
    % assign second image as initialization image
    I_init = I_i2;
    if params.through_gui
        gui_updateImage(I_init, gui_handles.ax_current_frame);
    end    
    
    % find 2D correspondences (sorted)
    [p_i1_uv, p_i2_uv, query_keypoints_vu] = findCorrespondeces(params,I_i1,I_i2);
    
    % homogenize keypoints
    p_hom_i1_uv = [p_i1_uv; ones(1,length(p_i1_uv))];
    p_hom_i2_uv = [p_i2_uv; ones(1,length(p_i2_uv))];    

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
    
    if params.init.use_BA
        [P_init, T_refined] = bundleAdjust(params, C1_P_hom_init(1:3,:), [p_hom_inlier_i1_uv(1:2,:); p_hom_inlier_i2_uv(1:2,:)], [T_WC1; T_WC2], K, 1);
        C1_P_hom_init(1:3,:) = P_init;
        
        % update homogeneous transformations
        T_WC1 = T_refined(1:4,1:4);
        T_WC2 = T_refined(5:8,1:4);
        T_C1W = tf2invtf(T_WC1);
        T_C1C2 = T_C1W*T_WC2;
        T_C2C1 = tf2invtf(T_C1C2);
    end
    
    % discard landmarks not contained in spherical neighborhood
    [C1_P_hom_init, outFOV_idx] = applySphericalFilter(params, C1_P_hom_init, params.init.landmarks_cutoff);
    % filter corresponding keypoints
    p_hom_inlier_i1_uv(:,outFOV_idx) = [];
    p_hom_inlier_i2_uv(:,outFOV_idx) = [];
    
    % assign initialization entities
    keypoints_init = flipud(p_hom_inlier_i2_uv(1:2,:));
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
                  size(keypoints_init,2), size(C2_landmarks_init,2)));
    
	% initialise keypoint tracker
    % create container for keypoint tracker (new keypoints with no landmarks)
    % set of candidate keypoints in last camera frame
    kp_tracks.candidate_kp = []; % 2xN
    % keypoint coordinates of every candiate in its first observed frame
    kp_tracks.first_obs_kp = [];  % 2xN
    % keypoint pose of every candiate in its first observed frame
    kp_tracks.first_obs_pose = []; % 16xN
    kp_tracks.nr_trackings = []; % 1xN
    
    kp_tracks_init = updateKpTracks(params, kp_tracks,I_i1, I_i2, query_keypoints_vu, T_WC2);    
end

% check for same number of keypoints and landmarks
assert(size(keypoints_init,2) == size(C2_landmarks_init,2));

end
