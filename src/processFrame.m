function [T_CiCj, p_new_matched_triang, updated_kp_tracks, Cj_corresponding_inlier_landmarks] =...
    processFrame(params,img_new,img_prev, keypoints_prev_triang, kp_tracks_old,Ci_landmarks_prev,T_WCi,K)
% Estimates pose transformation T_CiCj between to images.
% Tracks potential new keypoints and triangulates new landmarks if
% trianguability is good.
% 
% Input:
%  - params(struct) : parameter struct
%  - img_new(size) : current frame
%  - img_prev(size) : previous frame
%  - keypoints_prev_triang (2xN) : 2D points, [v u] which have
%    corresponding Landmarks
%  - kp_tracks_old : struct container for tracking keypoints
%  - Ci_landmarks_prev(3xN) : 3D points
%  - T_WCi : (4x4) Current transformation world to Ci
%  - K(3x3) : camera intrinsics matrix
%
% Output:
%  - T_CiCj(4x4) : transformation Cj to Ci
%  - p_new_matched_triang(2xN) : newly matched keypoints with 
%    corresponding landmarks, [v u] 
%  - updated_kp_tracks : struct updated container for tracking keypoints
%  - Cj_corresponding_inlier_landmarks (3xN) : 3D points in frame Cj
%    verified inliers by ransac

global fig_cont;

% show current frame
if params.cont.show_current_image
    figure(fig_cont);
    subplot(2,1,1);
    imshow(img_new);
    subplot(2,1,2);
    imshow(img_new);
end

%% Estimate delta rotation between frame i and j

[query_keypoints,matches] = ...
    findCorrespondeces_cont(params,img_prev,keypoints_prev_triang,img_new);

% delete landmark where no matching keypoint was found
corr_ldk_matches = matches(matches > 0);
Ci_corresponding_landmarks = Ci_landmarks_prev(:,corr_ldk_matches);

[~,matched_query_indices,matched_database_indices] = find(matches);

% sort query keypoints
matched_query_keypoints = query_keypoints(:,matched_query_indices);

% sort database keypoints
matched_database_keypoints = keypoints_prev_triang(:,matched_database_indices);

% check for consistent correspondences
assert(size(matched_query_keypoints,2) == length(corr_ldk_matches) && ...
       size(matched_database_keypoints,2) == length(corr_ldk_matches));

   % display matched keypoints
if params.localization_ransac.show_matched_keypoints
    figure(fig_cont);
    subplot(2,1,1);
    plotPoints(matched_query_keypoints,'g.');
    plotCircles(matched_query_keypoints,'y',params.localization_ransac.pixel_tolerance);
    title('Matched (green) keypoints with (yellow) confidence');
    
    subplot(2,1,2);
    plotPoints(matched_query_keypoints,'g.');
    title('Matched (green) keypoints');
end


% estimate pose from i to j
[R_CiCj,Ci_t_CiCj,p_new_matched_triang,Ci_corresponding_inlier_landmarks] = ...
    ransacLocalization(params,matched_query_keypoints,Ci_corresponding_landmarks,K);

if (~isempty(R_CiCj) && ~isempty(Ci_t_CiCj))
    fprintf('  >> Successfully localized\n');
else
    R_CiCj = eye(3,3);
    Ci_t_CiCj = zeros(3,1);
    fprintf('  No transformation found\n');
end



% construct new camera pose
T_CiCj = [R_CiCj   Ci_t_CiCj;
          zeros(1,3)       1];
      
T_CjCi = [R_CiCj'   -R_CiCj'*Ci_t_CiCj;
          zeros(1,3)                 1];


%% Candiate Keypoint tracker
% variable init - assume no matches
matches_untriang = zeros(1,size(query_keypoints,2));
updated_kp_tracks.candidate_kp = [];   % 2xN
updated_kp_tracks.first_obs_kp = [];   % 2xN
updated_kp_tracks.first_obs_pose = []; % 16xN

% descripe query keypoints
query_descriptors = describeKeypoints(img_new,query_keypoints,params.corr.descriptor_radius);

% if there are candiate keypoints, try to match
if (size(kp_tracks_old.candidate_kp,2) > 0) % 0 in first frame
    % describe database keypoints
    database_descriptors = describeKeypoints(img_prev,kp_tracks_old.candidate_kp,params.corr.descriptor_radius);

    % match descriptors
    matches_untriang = matchDescriptors(query_descriptors,database_descriptors,params.corr.match_lambda);

    % update candidate_kp coordinates with matched current kp
    idx_matched_kp_tracks_cand = matches_untriang(matches_untriang>0); % z.B 17
    % updated_kp_tracks.candidate_kp(:,idx_matched_kp_tracks_cand) = query_keypoints(:,matches_untriang>0); %new v,u coord
    
    % update kp information that could be tracked
    updated_kp_tracks.candidate_kp(:,idx_matched_kp_tracks_cand) = query_keypoints(:,idx_matched_kp_tracks_cand); %new v,u coord
    updated_kp_tracks.first_obs_kp(:,idx_matched_kp_tracks_cand) = kp_tracks_old.first_obs_kp(:,idx_matched_kp_tracks_cand);
    updated_kp_tracks.first_obs_pose(:,idx_matched_kp_tracks_cand) = kp_tracks_old.first_obs_pose(:,idx_matched_kp_tracks_cand);

    % discard kp that could not be tracked --> new sorting (unkown)
    updated_kp_tracks.candidate_kp = updated_kp_tracks.candidate_kp(:,idx_matched_kp_tracks_cand);
    updated_kp_tracks.first_obs_kp = updated_kp_tracks.first_obs_kp(:,idx_matched_kp_tracks_cand);
    updated_kp_tracks.first_obs_pose = updated_kp_tracks.first_obs_pose(:,idx_matched_kp_tracks_cand);
end

% append all new found keypoints and their pose
new_kp = query_keypoints(:,matches_untriang==0); % kp which could not be matched
T_WCi_col = T_WCi(:); % convert to col vector for storage
updated_kp_tracks.candidate_kp = [updated_kp_tracks.candidate_kp, new_kp];
updated_kp_tracks.first_obs_kp = [updated_kp_tracks.first_obs_kp, new_kp]; % is equal to candidate when adding
updated_kp_tracks.first_obs_pose = [updated_kp_tracks.first_obs_pose, repmat(T_WCi_col,[1, size(new_kp, 2)])];   

%% Triangulate new landmarks

% how to convert col back to matrix: matrix2D = reshape(columnVector, [m_rows m_columns]);
% TODO: calculate bearing vector from every candiate_kp

% TODO: If norm(bearing_vector) < trehshold --> triangulate new landmark

% % triangulate new points with keypoint
% Mi = K * eye(3,4);
% Mj = K * [R_CiCj, Ci_t_CiCj];kp
% p_hom_prev_matched = [p_prev_matched; ones(1,size(p_prev_matched,2))];
% p_hom_new_matched = [p_new_matched_triang; ones(1,size(p_new_matched_triang,2))];
% Ci_landmarks_new = linearTriangulation(p_hom_prev_matched,p_hom_new_matched,Mi,Mj);

%% Update keypoint tracks, Cj_landmarks and p_new_matched_triang

% TODO: Delete candidate keypoint used for triangulation from updated_kp_tracks

% TODO: Filter landmarks with cylindrical filter

% discard landmarks not contained in cylindrical neighborhood
% [Ci_landmarks_new, outFOV_idx] = applyCylindricalFilter(Ci_landmarks_new, params.cont.landmarks_cutoff);

% TODO: Delete keypoints of landmarks deleted by cylindrical filter
%p_i2(:,outFOV_idx) = [];

% TODO: Append used candidate keypoints to p_new_matched_triang

% TODO: Append landmarks in Cj-Frame at index corresponding to p_new_matched_triang
% Cj_P_hom_new = T_CjCi*[Ci_landmarks_new(1:3,:); ones(1,size(Ci_landmarks_new,2))];
% Cj_landmarks_updated = Cj_P_hom_new(1:3,:);
  
Cj_corresponding_inlier_landmarks = T_CjCi*[Ci_corresponding_inlier_landmarks(1:3,:); ones(1,size(Ci_corresponding_inlier_landmarks,2))];
Cj_corresponding_inlier_landmarks = Cj_corresponding_inlier_landmarks(1:3,:);


% % display statistics
% fprintf(['  Number of new landmarks triangulated: %i\n',...
%          '  Number of updated landmarks: %i\n'],...
%          size(Ci_landmarks_new,2), size(Cj_landmarks_updated,2));

end
