function [T_CiCj, p_new_matched_triang, updated_kp_tracks, Cj_new_landmarks] =...
    processFrame(params,img_new,img_prev, keypoints_prev_triang, kp_tracks_prev,Ci_landmarks_prev,T_WCi,K)
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

global fig_cont fig_kp_tracks;

% show current frame
if params.cont.show_current_image
    figure(fig_cont);
    subplot(2,1,1);
    imshow(img_new);
    subplot(2,1,2);
    imshow(img_new);
end

% show current frame
if params.keypoint_tracker.show_matches
    figure(fig_kp_tracks);
    subplot(2,1,1);
    hold on;
    imshow(img_new);
    subplot(2,1,2);
    hold on;
    imshow(img_new);
end

%% Estimate delta rotation from frame Cj to Ci
[query_keypoints,matches] = ...
    findCorrespondeces_cont(params,img_prev,keypoints_prev_triang,img_new);

% delete landmark where no matching keypoint was found
corr_ldk_matches = matches(matches > 0);
Ci_corresponding_landmarks = Ci_landmarks_prev(:,corr_ldk_matches);

[~,matched_query_indices,matched_database_indices] = find(matches);

% filter query keypoints
matched_query_keypoints = query_keypoints(:,matched_query_indices);

% filter database keypoints
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

% estimate pose from Cj to Ci
[R_CiCj,Ci_t_CiCj,p_new_matched_triang,Ci_corresponding_inlier_landmarks] = ...
    ransacLocalization(params,matched_query_keypoints,Ci_corresponding_landmarks,K);

localized = false;  % flag

if (isempty(R_CiCj) && isempty(Ci_t_CiCj))
    R_CiCj = eye(3,3);
    Ci_t_CiCj = zeros(3,1);
    fprintf(' !! No transformation found !!\n');
else
    fprintf('  >> Successfully localized\n');
    localized = true;
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


% if there are candiate keypoints, try to match
if (size(kp_tracks_prev.candidate_kp,2) > 0) % 0 in first frame
    % descripe query keypoints
    query_descriptors = describeKeypoints(img_new,query_keypoints,params.corr.descriptor_radius);

    % describe database keypoints
    database_descriptors = describeKeypoints(img_prev,kp_tracks_prev.candidate_kp,params.corr.descriptor_radius);

    % match descriptors
    matches_untriang = matchDescriptors(query_descriptors,database_descriptors,params.corr.match_lambda);
    % OPTIONAL TODO: Lucas kanade?

    % update candidate_kp coordinates with matched current kp
    idx_matched_kp_tracks_cand = matches_untriang(matches_untriang>0); % z.B 17

    % update kp information that could be tracked
    updated_kp_tracks.candidate_kp(:,idx_matched_kp_tracks_cand) = query_keypoints(:,idx_matched_kp_tracks_cand); %new v,u coord
    updated_kp_tracks.first_obs_kp(:,idx_matched_kp_tracks_cand) = kp_tracks_prev.first_obs_kp(:,idx_matched_kp_tracks_cand);
    updated_kp_tracks.first_obs_pose(:,idx_matched_kp_tracks_cand) = kp_tracks_prev.first_obs_pose(:,idx_matched_kp_tracks_cand);

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

% display matched keypoint tracks
if params.keypoint_tracker.show_matches
    figure(fig_kp_tracks);
    subplot(2,1,1);
    if (size(kp_tracks_prev.candidate_kp,2) > 0) % 0 in first frame
        plotPoints(kp_tracks_prev.candidate_kp,'r.');
    end
    % plotCircles(matched_query_keypoints,'y',params.localization_ransac.pixel_tolerance);
    title('Candidate Keypoints: Old (red)');

    subplot(2,1,2);
    if (size(kp_tracks_prev.candidate_kp,2) > 0) % 0 in first frame
        plotPoints(kp_tracks_prev.candidate_kp,'r.');
        plotMatches(matches_untriang,query_keypoints,kp_tracks_prev.candidate_kp,'m-');
    end
    plotPoints(updated_kp_tracks.candidate_kp,'y.');

    title('Candidate Keypoints: Old (red), updated (yellow), Matches');
end

fprintf('  Number of matched keypoint candidates: %i (%0.2f %%)\n'...
         ,nnz(matches_untriang),100*nnz(matches_untriang)/size(kp_tracks_prev.candidate_kp,2)); 

%% Triangulate new landmarks

% calculate bearing angle
vector_first = [(updated_kp_tracks.first_obs_kp);repmat(K(1,1),[1, size(updated_kp_tracks.first_obs_kp, 2)])];
vector_act = [updated_kp_tracks.candidate_kp;repmat(K(1,1),[1, size(updated_kp_tracks.candidate_kp, 2)])];

bearing_angle_d = atan2d(twoNormMatrix(cross(vector_act,vector_first)),dot(vector_act,vector_first));

% Create idx vector of trianguable candidate points
idx_good_triangable = (bearing_angle_d > 30); % to be tuned

p_candidates_first = updated_kp_tracks.first_obs_kp(:,idx_good_triangable);
p_candidates_first_pose = updated_kp_tracks.first_obs_pose(:,idx_good_triangable);
p_candidates_j = updated_kp_tracks.candidate_kp(:,idx_good_triangable);

p_hom_candidates_first = [p_candidates_first; ones(1,size(p_candidates_first,2))];
p_hom_candidates_j = [p_candidates_j; ones(1,size(p_candidates_j,2))];
Ci_P_hom_new = zeros(4,size(p_candidates_first,2));

fprintf('  Number of trianguable keypoint candidates: %i\n'...
         ,nnz(idx_good_triangable)); 

% Calculate M's
for i=1:size(p_candidates_first,2)
    T_WCfirst = reshape(p_candidates_first_pose(:,i), [4,4]);
    T_CfirstW = invTansformationMatrix(T_WCfirst);
    M_CfirstW = K * T_CfirstW(1:3,:); %T_WCfirst(1:3,:);  %eye(3,4);
           
    T_WCj = T_WCi * T_CiCj; % current pose against world 
    T_CjW = invTansformationMatrix(T_WCj);
    M_CjW = K * T_CjW(1:3,:); %T_WCj(1:3,:);

    % Calculate delta pose between Cfirst and Cj  
    T_Cfirst_Cj = T_CfirstW*T_WCj;
    M_CjCfirst = K * T_Cfirst_Cj(1:3,:); %[R_CiCj, Ci_t_CiCj];
    
    % Triangulate landmark
    Ci_P_hom_new(:,i) = linearTriangulation(p_hom_candidates_first(:,i),p_hom_candidates_j(:,i),M_CfirstW,M_CjCfirst);

end % for loop end

%% Update keypoint tracks, Cj_landmarks and p_new_matched_triang

% Delete candidate keypoint used for triangulation from updated_kp_tracks
updated_kp_tracks.candidate_kp = updated_kp_tracks.candidate_kp(:,~idx_good_triangable);
updated_kp_tracks.first_obs_kp = updated_kp_tracks.first_obs_kp(:,~idx_good_triangable);
updated_kp_tracks.first_obs_pose = updated_kp_tracks.first_obs_pose(:,~idx_good_triangable);

% Filter landmarks with cylindrical filter (still wrong frame??)
% [Cj_hom_landmarks_new, outFOV_idx] = applyCylindricalFilter(Cj_hom_landmarks_new, params.cont.landmarks_cutoff);
idx_Ci_P_hom_new_realistic = find(Ci_P_hom_new(3,:)>0);

Cj_P_hom_new = [];
% Remove unrealistic landmarks and corresponding keypoints
if (nnz(idx_Ci_P_hom_new_realistic)>0)
    Ci_P_hom_new = Ci_P_hom_new(:,idx_Ci_P_hom_new_realistic);
    p_candidates_j = p_candidates_j(:,idx_Ci_P_hom_new_realistic);
    
    % calculate new landmarks in Cj-Frame
    Cj_P_hom_new = T_CjCi*Ci_P_hom_new;
else
    fprintf('None of the triangulated landmarks was realistic!\n')
    p_candidates_j = [];
end

% Append used candidate keypoints to p_new_matched_triang
p_new_matched_triang = [p_new_matched_triang, p_candidates_j];

Cj_P_hom_inliers = [];
if localized %otherwise index error since Ci_corresponding_inlier_landmarks = []
    Cj_P_hom_inliers = T_CjCi*[Ci_corresponding_inlier_landmarks; ones(1,size(Ci_corresponding_inlier_landmarks,2))];
end
    
Cj_P_hom = [Cj_P_hom_inliers, Cj_P_hom_new];
Cj_new_landmarks = Cj_P_hom(1:3,:);

%% display statistics

fprintf('  Number of landmarks (total/new): %i / %i\n'...
         ,size(Cj_new_landmarks,2), size(Cj_P_hom_new,2)); 

% fprintf(['  Number of new landmarks triangulated: %i\n',...
%          '  Number of updated landmarks: %i\n'],...
%          size(Ci_landmarks_new,2), size(Cj_landmarks_updated,2));


end

