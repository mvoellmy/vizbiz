function [T_CiCj, p_new_matched_triang, kp_tracks_updated, Cj_new_landmarks, reInitFlag] =...
    processFrame(params, img_new, img_prev, img_reInit, T_WCinit, keypoints_prev_triang, kp_tracks_prev, Ci_landmarks_prev, T_WCi, K)
% Estimates pose transformation T_CiCj between two images.
% Tracks potential new keypoints and triangulates new landmarks if
% trianguability is good.
% 
% Input:
%  - params(struct) : parameter struct
%  - img_new(size) : current frame
%  - img_prev(size) : previous frame
%  - img_reInit(size) : image for reinit
%  - T_WCinit (4x4) : Transformation world to Cinit
%  - keypoints_prev_triang (2xN) : 2D points, [v u] which have
%    corresponding Landmarks
%  - kp_tracks_old : struct container for tracking keypoints (no associated
%    landmarks)
%  - Ci_landmarks_prev(3xN) : 3D points
%  - T_WCi : (4x4) Current transformation world to Ci
%  - K(3x3) : camera intrinsics matrix
%
% Output:
%  - T_CiCj(4x4) : transformation Cj to Ci
%  - p_new_matched_triang(2xN) : newly matched keypoints with 
%    corresponding landmarks, [v u] 
%  - updated_kp_tracks : struct updated container for tracking keypoints (no associated
%    landmarks)
%  - Cj_new_landmarks (3xN) : 3D points in frame Cj
%    verified inliers by ransac + new triangulated landmarks
%  - reInitFlag (bool) : Flag true when reInit was performed
% todo

global fig_cont fig_kp_tracks fig_kp_triangulate gui_handles;

% show current frame
if (params.cont.figures && params.cont.show_new_image)
    figure(fig_cont);
    subplot(2,1,1);
    imshow(img_new);
    subplot(2,1,2);
    imshow(img_new);
end

% show current frame
if params.cont.figures
    figure(fig_kp_tracks);
    imshow(img_new);
    hold on;
end

% show current frame
if params.cont.figures
    figure(fig_kp_triangulate);
    clf;
    imshow(img_new);
    hold on;
end

%% Find correspondences between new and prev image
[query_keypoints, matched_query_indices, matched_query_keypoints, Ci_corresponding_landmarks] = ...
    findCorrespondeces_cont(params, img_prev, keypoints_prev_triang, img_new, Ci_landmarks_prev);

% check for consistent correspondences
assert(size(matched_query_keypoints,2) == size(Ci_corresponding_landmarks,2));

% display fraction of matched keypoints/landmarks
updateConsole(params,...
              sprintf('  Number of new keypoints matched with prev keypoints by descriptor or klt: %i (%0.2f perc.)\n',...
              size(matched_query_keypoints,2),100*size(matched_query_keypoints,2)/size(keypoints_prev_triang,2)));

% display matched keypoints
if (params.cont.figures && params.localization_ransac.show_matched_keypoints)
    figure(fig_cont);
    subplot(2,1,1);
    plotPoints(matched_query_keypoints,'g.');
    plotCircles(matched_query_keypoints,'y',params.localization_ransac.pixel_tolerance);
    title('Matched kp (green) with confidence (yellow)');
    
    subplot(2,1,2);
    plotPoints(matched_query_keypoints,'g.');
    title('Matched kp (green)');
end

%% Estimate transformation Cj to Ci
[R_CiCj, Ci_t_CiCj, p_new_matched_triang, Ci_corresponding_inlier_landmarks] = ...
    p3pRansac(params, matched_query_keypoints, Ci_corresponding_landmarks, K);

% check number of inlier landmarks
if (size(Ci_corresponding_inlier_landmarks,2) > params.cont.reinit.inlier_th )
    
    reInitFlag = false;

    % set flag
    localized = false;

    if (isempty(R_CiCj) && isempty(Ci_t_CiCj))
        R_CiCj = eye(3,3);
        Ci_t_CiCj = zeros(3,1);
        updateConsole(params, '  !! No transformation found !!\n');
    else
        updateConsole(params, '  >> Successfully localized\n');
        localized = true;
    end

    % construct new camera pose
    T_CiCj = [R_CiCj   Ci_t_CiCj;
              zeros(1,3)       1];
    T_CjCi = tf2invtf(T_CiCj);

    %% Candiate Keypoint tracker
    T_WCj = T_WCi * T_CiCj;
    unmatched_query_kp = query_keypoints;
    unmatched_query_kp(:,matched_query_indices>0) = [];
    kp_tracks_updated = updateKpTracks(params, kp_tracks_prev, img_prev, img_new, unmatched_query_kp, T_WCj);

    %% Triangulate new landmarks & update landmarks and keypoint list
    nr_landmarks = size(Ci_corresponding_inlier_landmarks, 2);
    if params.kp_tracker.min_nr_landmarks > nr_landmarks
        [Cj_P_hom_new_inliers, p_candidates_j_inliers, kp_tracks_updated] =...
            triangulateNewLandmarks(params, kp_tracks_updated, K , fig_kp_triangulate, fig_kp_tracks, T_WCj, nr_landmarks);
    else
        Cj_P_hom_new_inliers = [];
        p_candidates_j_inliers = [];
    end

    % append used candidate keypoints to p_new_matched_triang
    p_new_matched_triang = [p_new_matched_triang, p_candidates_j_inliers];

    Cj_P_hom_inliers = [];
    if localized % otherwise index error since Ci_corresponding_inlier_landmarks = []
        Cj_P_hom_inliers = T_CjCi * [Ci_corresponding_inlier_landmarks; ones(1,size(Ci_corresponding_inlier_landmarks,2))];
    end

    Cj_P_hom = [Cj_P_hom_inliers, Cj_P_hom_new_inliers];
    Cj_new_landmarks = [];
    if (size(Cj_P_hom,2)>0)
        Cj_new_landmarks = Cj_P_hom(1:3,:);
    end

    % update gui triangulated keypoints
    if params.through_gui && params.gui.show_triang_features
        gui_updateKeypoints(p_new_matched_triang, gui_handles.ax_current_frame, 'gx');
    end

    % display statistics
    updateConsole(params,...
                  sprintf('  Number of landmarks (total/new): %i / %i\n',...
                  size(Cj_new_landmarks,2), size(Cj_P_hom_new_inliers,2)));
else
    % do reinitialization
    if (params.cont.reinit.do_reinit)
        updateConsole(params,...
                  sprintf('  ###########################\n Reinitialization performed!\n'));
        
        % set flag
        reInitFlag = true;
        % reinit pipeline
        [~, keypoints_reInit, Cj_landmarks_reInit, T_CinitCj, kp_tracks_reInit] =...
            initPipeline(params, img_reInit, img_new, K, T_WCinit);
        kp_tracks_updated = kp_tracks_reInit;
        Cj_new_landmarks = Cj_landmarks_reInit; 
        p_new_matched_triang = keypoints_reInit;
        T_CiCj = T_CinitCj;
    end
end

 end