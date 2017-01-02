function [T_CiCj, state, kp_tracks_updated] = processFrame(params, img_new, img_prev, state_prev, kp_tracks_prev, K)
% Estimates pose transformation T_CiCj between two images.
% Tracks potential new keypoints and triangulates new landmarks if
% trianguability is good.
% 
% Input:
%  - params(struct) : parameter struct
%  - img_new(size) : current frame
%  - img_prev(size) : previous frame
%  - state_prev(struct) : previous state
%  - K(3x3) : camera intrinsics matrix
%  - kp_tracks_prev
%
% Output:
%  - T_CiCj(4x4) : transformation Cj to Ci
%  - state(struct) : new state
%  - kp_tracks_updated

global fig_cont fig_kp_tracks fig_kp_triangulate;

% show current frame
if params.cont.show_current_image
    figure(fig_cont);
    subplot(3,1,1);
    imshow(img_new);
    subplot(3,1,2);
    imshow(img_new);
end

% show current frame
if params.keypoint_tracker.show_matches
    figure(fig_kp_tracks);
    % subplot(2,1,1);
    % hold on;
    % imshow(img_new);
    % subplot(2,1,2);
    
    imshow(img_new);
    hold on;
end

% show current frame
if params.keypoint_tracker.show_triangulated
    figure(fig_kp_triangulate);
    clf;
    imshow(img_new);
    hold on;
end

%% Find correspondences
[query_keypoints, matches_untriang, matched_query_keypoints, matched_database_keypoints] = ...
    findCorrespondeces_cont(params, img_prev, state_prev.keypoints, img_new);

% display all correspondences
if params.cont.show_keypoints
    figure(fig_cont);
    subplot(3,1,1);
    hold on;
    plotPoints(query_keypoints,'r.');
    title('New keypoints');
end

% remove landmark where no matching keypoint was found
corr_ldk_matches = matches_untriang(matches_untriang > 0);
Ci_corresponding_landmarks = state_prev.Ci_landmarks(:,corr_ldk_matches);

% check for consistent correspondences
assert(size(matched_query_keypoints,2) == length(corr_ldk_matches) && ...
       size(matched_database_keypoints,2) == length(corr_ldk_matches));

% display matched keypoints
if params.cont.show_keypoints
    figure(fig_cont);
    subplot(3,1,1);
    plotPoints(matched_query_keypoints,'g.');
    plotCircles(matched_query_keypoints,'y',params.localization_ransac.pixel_tolerance);
    title('Matched (green) keypoints with (yellow) confidence');
    
    if params.cont.show_matches
        subplot(3,1,2);
        showMatchedFeatures(img_prev, img_new,...
                            flipud(matched_query_keypoints)',...
                            flipud(matched_database_keypoints)', 'blend', 'PlotOptions', {'rx','r.','m-'});
        title('Candidate keypoint matches');
    end
end

%% Estimate transformation Cj to Ci
[R_CiCj, Ci_t_CiCj, matched_query_inlier_keypoints, Ci_corresponding_inlier_landmarks, inliers] = ...
    p3pRansac(params, matched_query_keypoints, Ci_corresponding_landmarks, K);

% extract inlier keypoints
matched_database_inlier_keypoints = matched_database_keypoints(:,inliers);

% show inlier and filtered matches
if params.cont.show_inlier_matches
    figure(fig_cont);
    subplot(3,1,3);
    showMatchedFeatures(img_prev, img_new,...
                        flipud(matched_database_inlier_keypoints)',...
                        flipud(matched_query_inlier_keypoints)', 'blend', 'PlotOptions', {'r.','g.','y-'});
    title('Filtered inlier keypoint matches');
end

localized = false;

if (isempty(R_CiCj) && isempty(Ci_t_CiCj))
    R_CiCj = eye(3,3);
    Ci_t_CiCj = zeros(3,1);
    updateConsole(params, ' !! No transformation found !!\n');
else
    updateConsole(params, '  >> Successfully localized\n');
    localized = true;
end

% construct new camera pose
T_CiCj = [R_CiCj   Ci_t_CiCj;
          zeros(1,3)       1];
T_CjCi = tf2invtf(T_CiCj);


%% Propagate state (j)
state.keypoints = matched_query_inlier_keypoints;
Cj_P_hom_inlier = T_CjCi * [Ci_corresponding_inlier_landmarks; ones(1,size(Ci_corresponding_inlier_landmarks,2))];
state.Cj_landmarks = Cj_P_hom_inlier(1:3,:);
state.T_WCj = state_prev.T_WCi * T_CiCj;

%% Candiate Keypoint tracker

T_WCj = T_WCi*T_CiCj;
kp_tracks_updated = update_kp_tracks(params, kp_tracks_prev,img_prev, img_new, query_keypoints, T_WCj);

%% Triangulate new landmarks & update landmarks and keypoint list

[ Cj_P_hom_new_inliers, p_candidates_j_inliers, kp_tracks_updated ] =...
    triangulate_new_landmarks(params, kp_tracks_updated, K , fig_kp_triangulate, fig_kp_tracks, T_WCj);

% Append used candidate keypoints to p_new_matched_triang
p_new_matched_triang = [state.keypoints, p_candidates_j_inliers];

Cj_P_hom_inliers = [];
if localized %otherwise index error since Ci_corresponding_inlier_landmarks = []
    Cj_P_hom_inliers = T_CjCi*[Ci_corresponding_inlier_landmarks; ones(1,size(Ci_corresponding_inlier_landmarks,2))];
end

Cj_P_hom = [Cj_P_hom_inliers, Cj_P_hom_new_inliers];
Cj_new_landmarks = [];
if (size(Cj_P_hom,2)>0)
    Cj_new_landmarks = Cj_P_hom(1:3,:);
end

%% display statistics

fprintf('  Number of landmarks (total/new): %i / %i\n'...
         ,size(Cj_new_landmarks,2), size(Cj_P_hom_new_inliers,2)); 

end

