function [T_CiCj, state] = processFrame(params, img_new, img_prev, state_prev, K)
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
%
% Output:
%  - T_CiCj(4x4) : transformation Cj to Ci
%  - state(struct) : new state

global fig_cont;

% show current frame
if params.cont.show_current_image
    figure(fig_cont);
    subplot(3,1,1);
    imshow(img_new);
    subplot(3,1,2);
    imshow(img_new);
end

%% Find correspondences
[query_keypoints, matches, matched_query_keypoints, matched_database_keypoints] = ...
    findCorrespondeces_cont(params, img_prev, state_prev.keypoints, img_new);

% display all correspondences
if params.cont.show_keypoints
    figure(fig_cont);
    subplot(3,1,1);
    hold on;
    plotPoints(query_keypoints,'r.');
    title('New keypoints');
end

% % filter query keypoints and database keypoints
% [~, matched_query_indices, matched_database_indices] = find(matches);
% matched_query_keypoints = query_keypoints(:,matched_query_indices);
% matched_database_keypoints = state_prev.keypoints(:,matched_database_indices);

% remove landmark where no matching keypoint was found
corr_ldk_matches = matches(matches > 0);
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
% variable init - assume no matches
matches_untriang = zeros(1,size(query_keypoints,2));
kp_tracks_updated.candidate_kp = [];   % 2xN
kp_tracks_updated.first_obs_kp = [];   % 2xN
kp_tracks_updated.first_obs_pose = []; % 16xN
kp_tracks_updated.nr_trackings = []; % 1xN

% if there are candiate keypoints, try to match
if (size(kp_tracks_prev.candidate_kp,2) > 0) % 0 in first frame
    % descripe query keypoints
    query_descriptors = describeKeypoints(img_new,query_keypoints,params.corr.descriptor_radius);

    % describe database keypoints
    database_descriptors = describeKeypoints(img_prev,kp_tracks_prev.candidate_kp,params.corr.descriptor_radius);
    
    % match descriptors
    matches_untriang = matchDescriptors(query_descriptors,database_descriptors,params.corr.match_lambda);
    % OPTIONAL TODO: Lucas kanade?
    
    new_kp_1 = query_keypoints(:,matches_untriang==0);
    new_kp_2 = [];
    
%     fprintf('------------>Number of keypoints before RANSAC: %d\n', nnz(matches_untriang));
%      [query_keypoints, kp_tracks_prev.candidate_kp, matches_untriang, new_kp_2] = ...
%          eightPointRansac_cont(params, query_keypoints, kp_tracks_prev.candidate_kp, matches_untriang, K, K);
%     fprintf('------------>Number of keypoints after  RANSAC: %d\n', nnz(matches_untriang));
%  
    % update candidate_kp coordinates with matched current kp
    idx_matched_kp_tracks_cand = find(matches_untriang); % z.B 17
    idx_matched_kp_tracks_database = matches_untriang(matches_untriang>0);

    % update kp information that could be tracked (sorting like query keypoints)
    kp_tracks_updated.candidate_kp(:,idx_matched_kp_tracks_cand) = query_keypoints(:,idx_matched_kp_tracks_cand); %new v,u coord
    kp_tracks_updated.first_obs_kp(:,idx_matched_kp_tracks_cand) = kp_tracks_prev.first_obs_kp(:,idx_matched_kp_tracks_database);
    kp_tracks_updated.first_obs_pose(:,idx_matched_kp_tracks_cand) = kp_tracks_prev.first_obs_pose(:,idx_matched_kp_tracks_database);
    kp_tracks_updated.nr_trackings(idx_matched_kp_tracks_cand) = kp_tracks_prev.nr_trackings(idx_matched_kp_tracks_database)+1;

    % discard kp that could not be tracked --> new sorting (unkown)
    kp_tracks_updated.candidate_kp = kp_tracks_updated.candidate_kp(:,idx_matched_kp_tracks_cand);
    kp_tracks_updated.first_obs_kp = kp_tracks_updated.first_obs_kp(:,idx_matched_kp_tracks_cand);
    kp_tracks_updated.first_obs_pose = kp_tracks_updated.first_obs_pose(:,idx_matched_kp_tracks_cand);
    kp_tracks_updated.nr_trackings = kp_tracks_updated.nr_trackings(idx_matched_kp_tracks_cand);

    % append all new found keypoints and their pose
    new_kp = [new_kp_1, new_kp_2]; % kp which could not be matched
else
    new_kp = query_keypoints(:,matches_untriang==0);
end

T_WCj = T_WCi * T_CiCj;
T_WCj_col = T_WCj(:); % convert to col vector for storage
kp_tracks_updated.candidate_kp = [kp_tracks_updated.candidate_kp, new_kp];
kp_tracks_updated.first_obs_kp = [kp_tracks_updated.first_obs_kp, new_kp]; % is equal to candidate when adding
kp_tracks_updated.first_obs_pose = [kp_tracks_updated.first_obs_pose, repmat(T_WCj_col,[1, size(new_kp, 2)])];
kp_tracks_updated.nr_trackings = [kp_tracks_updated.nr_trackings, zeros(1, size(new_kp, 2))];

% display matched keypoint tracks
if params.keypoint_tracker.show_matches
    figure(fig_kp_tracks);
%     subplot(2,1,1);
%     if (size(kp_tracks_prev.candidate_kp,2) > 0) % 0 in first frame
%         plotPoints(kp_tracks_prev.candidate_kp,'r.');
%     end
%     title('Candidate Keypoints: Old (red)');

%     subplot(2,1,2);
    if (size(kp_tracks_prev.candidate_kp,2) > 0) % 0 in first frame
        plotPoints(kp_tracks_prev.candidate_kp,'r.');
        plotMatches(matches_untriang,query_keypoints,kp_tracks_prev.candidate_kp,'m-');
    end
    plotPoints(kp_tracks_updated.candidate_kp,'y.');

    %title('Candidate Keypoints: Old (red), updated (yellow), Matches');
end

fprintf('  Number of matched keypoint candidates: %i (%0.2f %%)\n'...
         ,nnz(matches_untriang),100*nnz(matches_untriang)/size(kp_tracks_prev.candidate_kp,2)); 

%% Triangulate new landmarks
% calculate bearing angle
vector_first = [(kp_tracks_updated.first_obs_kp);repmat(K(1,1),[1, size(kp_tracks_updated.first_obs_kp, 2)])];
vector_act = [kp_tracks_updated.candidate_kp;repmat(K(1,1),[1, size(kp_tracks_updated.candidate_kp, 2)])];

bearing_angle_d = atan2d(twoNormMatrix(cross(vector_act,vector_first)),dot(vector_act,vector_first));

% create idx vector of trianguable candidate points
idx_good_trianguable = ((bearing_angle_d > params.keypoint_tracker.bearing_low_thr)...
    & (kp_tracks_updated.nr_trackings >= params.keypoint_tracker.min_nr_trackings));

p_candidates_first = kp_tracks_updated.first_obs_kp(:,idx_good_trianguable);
p_candidates_first_pose = kp_tracks_updated.first_obs_pose(:,idx_good_trianguable);
p_candidates_j = kp_tracks_updated.candidate_kp(:,idx_good_trianguable);

p_hom_candidates_first = [p_candidates_first; ones(1,size(p_candidates_first,2))];
p_hom_candidates_j = [p_candidates_j; ones(1,size(p_candidates_j,2))];

% Variable init
Cfirst_P_hom_new = zeros(4,size(p_candidates_first,2));
Cj_P_hom_new = zeros(4,size(p_candidates_first,2));

fprintf('  Number of trianguable keypoint candidates: %i\n'...
         ,nnz(idx_good_trianguable)); 

% Show matches from first and j image of keypoints
if params.keypoint_tracker.show_matches
    figure(fig_kp_tracks);
%     subplot(2,1,1);
%     if (size(kp_tracks_prev.candidate_kp,2) > 0) % 0 in first frame
%         plotPoints(kp_tracks_prev.candidate_kp,'r.');
%     end
%     title('Candidate Keypoints: Old (red)');

%     subplot(2,1,2);
    if (size(p_hom_candidates_first,2) > 0) % 0 in first frame
        plotPoints(p_hom_candidates_first(1:2,:),'gx');
        plotPoints(p_hom_candidates_j(1:2,:),'bx');
        plotMatches(1:size(p_hom_candidates_j,2),p_hom_candidates_j,p_hom_candidates_first,'g-');     
    end
    title('Candiate Keypoints: Prev image (red), j-Image (yellow), Trianguable First image (gx), Trianguable j-Image (bx)');
end
     
     
% Linear triangulation
for i=1:size(p_candidates_first,2)
    T_WCfirst = reshape(p_candidates_first_pose(:,i), [4,4]);
    
    % Calculate M's
    M_Cfirst = K * eye(3,4); %T_CfirstW(1:3,:);

    % Calculate delta pose between Cfirst and Cj  
    T_CjW = tform2invtform(T_WCj);
    T_CjCfirst = T_CjW * T_WCfirst;
    M_CjCfirst = K * T_CjCfirst(1:3,:);
        
    % Triangulate landmark
    Cfirst_P_hom_new(:,i) = linearTriangulation(p_hom_candidates_first(:,i),p_hom_candidates_j(:,i),M_Cfirst,M_CjCfirst);
    Cj_P_hom_new(:,i) = T_CjCfirst*Cfirst_P_hom_new(:,i);
end


%% Update keypoint tracks, Cj_landmarks and p_new_matched_triang

% Delete candidate keypoint used for triangulation from updated_kp_tracks
kp_tracks_updated.candidate_kp = kp_tracks_updated.candidate_kp(:,~idx_good_trianguable);
kp_tracks_updated.first_obs_kp = kp_tracks_updated.first_obs_kp(:,~idx_good_trianguable);
kp_tracks_updated.first_obs_pose = kp_tracks_updated.first_obs_pose(:,~idx_good_trianguable);
kp_tracks_updated.nr_trackings = kp_tracks_updated.nr_trackings(~idx_good_trianguable);

% Filter landmarks with cylindrical filter (still wrong frame??)
% [Cj_hom_landmarks_new, outFOV_idx] = applyCylindricalFilter(Cj_hom_landmarks_new, params.cont.landmarks_cutoff);
idx_Ci_P_hom_new_realistic = find(Cj_P_hom_new(3,:)>0);

% Remove unrealistic landmarks and corresponding keypoints
if (nnz(idx_Ci_P_hom_new_realistic)>0)
    Cj_P_hom_new = Cj_P_hom_new(:,idx_Ci_P_hom_new_realistic);
    p_candidates_j = p_candidates_j(:,idx_Ci_P_hom_new_realistic);

else
    fprintf('None of the triangulated landmarks was realistic!\n')
    p_candidates_j = [];
end

% Append used candidate keypoints to p_new_matched_triang
matched_query_inlier_keypoints = [matched_query_inlier_keypoints, p_candidates_j];

Cj_P_hom_inliers = [];
if localized %otherwise index error since Ci_corresponding_inlier_landmarks = []
    Cj_P_hom_inliers = T_CjCi*[Ci_corresponding_inlier_landmarks; ones(1,size(Ci_corresponding_inlier_landmarks,2))];
end
    
Cj_P_hom = [Cj_P_hom_inliers, Cj_P_hom_new];
Cj_new_landmarks = Cj_P_hom(1:3,:);


%% display triangulated backprojected keypoints
Cj_projected_points = projectPoints(Cj_P_hom_new(1:3,:), K);
% Cj_projected_points = projectPoints((R_CjCi*Cj2_P_new) +...
%                                      repmat(-Cj_t_CjCi,[1 size(Cj2_P_new, 2)]),K);

if params.keypoint_tracker.show_triangulated
    good_idx_match = 1:size(Cj_projected_points,2);
    figure(fig_kp_triangulate);
    if (size(p_candidates_j,2) > 0) % 0 in first frame
        plotPoints(p_candidates_j,'r.');
        plotPoints(flipud(Cj_projected_points),'gx');
        plotMatches(good_idx_match,p_candidates_j,flipud(Cj_projected_points),'m-');
        % plotPoints(projected_points,'g.');
    end
    % plotPoints(updated_kp_tracks.candidate_kp,'y.');

    title('Triangable query KP (red), reprojected generated landmarks (green)');
    hold off;
end

% display statistics
fprintf('  Number of landmarks (total/new): %i / %i\n',...
        size(Cj_new_landmarks,2), size(Cj_P_hom_new,2)); 

end
