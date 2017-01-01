function [ Cj_P_hom_new_inliers, p_candidates_j_inliers, kp_tracks_updated ] =...
    triangulate_new_landmarks(params, kp_tracks_updated, K , fig_kp_triangulate, fig_kp_tracks, T_WCj)
% Checks which candiate keypoints have good trianguability (bearing vector)
% and then triangulates a new landmark.
% Newly triangulated landmarks are filtered to have positive z-coordinate
% and their reprojetion error must be smaller then a defined threshold.
% The used candidate keypoints are removed from the keypoint tracks and the
% cleaned up keypoints tracks is returned.

% Outputs:

% - Cj_P_hom_new_inliers [3xN]: New landmarks filtered
% - p_candidates_j_inliers [2xN]: Keypoints corresponding to landmarks [v u]

%% Triangulate new landmarks
% calculate bearing angle
vector_first = [(kp_tracks_updated.first_obs_kp);repmat(K(1,1),[1, size(kp_tracks_updated.first_obs_kp, 2)])];
vector_j = [kp_tracks_updated.candidate_kp;repmat(K(1,1),[1, size(kp_tracks_updated.candidate_kp, 2)])];

bearing_angle_d = atan2d(twoNormMatrix(cross(vector_j,vector_first)),dot(vector_j,vector_first));

% Create idx vector of trianguable candidate points
idx_good_trianguable = ((bearing_angle_d > params.keypoint_tracker.bearing_low_thr)...
    & (kp_tracks_updated.nr_trackings >= params.keypoint_tracker.min_nr_trackings)...
    & (bearing_angle_d < params.keypoint_tracker.bearing_up_thr));

p_candidates_first = kp_tracks_updated.first_obs_kp(:,idx_good_trianguable);
p_candidates_first_pose = kp_tracks_updated.first_obs_pose(:,idx_good_trianguable);
p_candidates_j = kp_tracks_updated.candidate_kp(:,idx_good_trianguable);

% Show matches from first and j image of keypoints
if params.keypoint_tracker.show_matches
    figure(fig_kp_tracks);
    if (size(p_candidates_first,2) > 0) % 0 in first frame
        plotPoints(p_candidates_first(1:2,:),'gx');
        plotPoints(p_candidates_j(1:2,:),'bx');
        plotMatches(1:size(p_candidates_j,2),p_candidates_j,p_candidates_first,'g-');     
    end
    title('Candiate Keypoints: Prev image (red), j-Image (yellow), Trianguable First image (gx), Trianguable j-Image (bx)');
end

p_hom_candidates_first_uv = [flipud(p_candidates_first); ones(1,size(p_candidates_first,2))];
p_hom_candidates_j_uv = [flipud(p_candidates_j); ones(1,size(p_candidates_j,2))];

% Variable init
Cfirst_P_hom_new = zeros(4,size(p_candidates_first,2));
Cj_P_hom_new = zeros(4,size(p_candidates_first,2));

fprintf('  Number of trianguable keypoint candidates: %i\n'...
         ,nnz(idx_good_trianguable)); 
     
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
    Cfirst_P_hom_new(:,i) = linearTriangulation(p_hom_candidates_first_uv(:,i),p_hom_candidates_j_uv(:,i),M_Cfirst,M_CjCfirst);
    Cj_P_hom_new(:,i) = T_CjCfirst*Cfirst_P_hom_new(:,i);

end % for loop end

%% Update keypoint tracks, Cj_landmarks and p_new_matched_triang

% Delete candidate keypoint used for triangulation from updated_kp_tracks
kp_tracks_updated.candidate_kp = kp_tracks_updated.candidate_kp(:,~idx_good_trianguable);
kp_tracks_updated.first_obs_kp = kp_tracks_updated.first_obs_kp(:,~idx_good_trianguable);
kp_tracks_updated.first_obs_pose = kp_tracks_updated.first_obs_pose(:,~idx_good_trianguable);
kp_tracks_updated.nr_trackings = kp_tracks_updated.nr_trackings(~idx_good_trianguable);

%% Filter landmarks with 'cylindrical' and reprojection filter
% [Cj_hom_landmarks_new, outFOV_idx] = applyCylindricalFilter(Cj_hom_landmarks_new, params.cont.landmarks_cutoff);
idx_Ci_P_hom_new_realistic = find(Cj_P_hom_new(3,:)>0);

Cj_reprojected_points_uv = [];
Cj_P_hom_new_inliers = [];
p_candidates_j_inliers = [];

% Remove unrealistic landmarks and corresponding keypoints
if (nnz(idx_Ci_P_hom_new_realistic)>0)
    Cj_P_hom_new = Cj_P_hom_new(:,idx_Ci_P_hom_new_realistic);
    p_candidates_j = p_candidates_j(:,idx_Ci_P_hom_new_realistic);
    
    % Reproject realistic landmarks to remove outliers
    Cj_reprojected_points_uv = projectPoints(Cj_P_hom_new(1:3,:), K);
    difference = p_candidates_j - flipud(Cj_reprojected_points_uv);
    errors = sum(difference.^2, 1);
    reproj_inliers = errors < params.keypoint_tracker.max_reproj_error^2;

    Cj_P_hom_new_inliers = Cj_P_hom_new(:, reproj_inliers);
    p_candidates_j_inliers = p_candidates_j(:, reproj_inliers);
    fprintf('  Removed %i of %i realistic landmarks due to too big reprojection error\n', nnz(~reproj_inliers), size(Cj_P_hom_new,2));
else
    fprintf('None of the triangulated landmarks was realistic!\n')
    p_candidates_j = [];
end

%% display triangulated backprojected keypoints

if params.keypoint_tracker.show_triangulated
    good_idx_match = 1:size(Cj_reprojected_points_uv,2);
    figure(fig_kp_triangulate);
    if (size(p_candidates_j,2) > 0) % 0 in first frame
        plotPoints(p_candidates_j,'r.');
        plotPoints(flipud(Cj_reprojected_points_uv),'gx');
        plotMatches(good_idx_match,p_candidates_j,flipud(Cj_reprojected_points_uv),'m-');
        % plotPoints(projected_points,'g.');
    end
    
    title('Triangable query KP (red), reprojected generated landmarks (green)');
    hold off;
end

end

