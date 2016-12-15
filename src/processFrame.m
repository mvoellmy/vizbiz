function [T_CiCj, p_new_matched_triang, updated_keypoint_tracker, Cj_corresponding_inlier_landmarks] =...
    processFrame(params,img_new,img_prev, keypoints_prev_triang, keypoints_tracker_old,Ci_landmarks_prev,K)
% TODO description
% 
% Input:
%  - params(struct) : parameter struct
%  - img_new(size) : current frame
%  - img_prev(size) : previous frame
%  - keypoints_prev_triang (2xN) : 2D points, [v u] which have
%    corresponding Landmarks
%  - keypoints_tracker : container for tracking keypoints NOT DEFINED YET
%  - Ci_landmarks_prev(3xN) : 3D points
%  - K(3x3) : camera intrinsics matrix
%
% Output:
%  - T_CiCj(4x4) : transformation Cj to Ci
%  - p_new_matched_triang(2xN) : newly matched keypoints with 
%    corresponding landmarks, [v u] 
%  - updated_keypoint_tracker : updated container for tracking keypoints NOT DEFINED YET
%  - Cj_landmarks_updated(3xN) : 3D points in frame Cj

global fig_cont;

% show current frame
if params.cont.show_current_image
    figure(fig_cont);
    subplot(2,1,1);
    imshow(img_new);
    subplot(2,1,2);
    imshow(img_new);
end


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


% descripe query keypoints
query_descriptors = describeKeypoints(img_new,query_keypoints,params.corr.descriptor_radius);

% describe database keypoints
%database_descriptors = describeKeypoints(img_prev,keypoints_tracker_old,params.corr.descriptor_radius);

% match descriptors
%matches_untriang = matchDescriptors(query_descriptors,database_descriptors,params.corr.match_lambda);

% discard kp that could not be tracked
% [~,matched_query_indices,keypoints_prev_untriang_indices] = find(matches_untriang);
% keypoints_prev_untriang = keypoints_prev_untriang(matches_untriang);

% TODO: track these matches


% TODO: triangulate new LM if track good
      
% % triangulate new points with keypoint
% Mi = K * eye(3,4);
% Mj = K * [R_CiCj, Ci_t_CiCj];
% p_hom_prev_matched = [p_prev_matched; ones(1,size(p_prev_matched,2))];
% p_hom_new_matched = [p_new_matched_triang; ones(1,size(p_new_matched_triang,2))];
% Ci_landmarks_new = linearTriangulation(p_hom_prev_matched,p_hom_new_matched,Mi,Mj);

% discard landmarks not contained in cylindrical neighborhood
% [Ci_landmarks_new, outFOV_idx] = applyCylindricalFilter(Ci_landmarks_new, params.cont.landmarks_cutoff);

% todo: remove corresponding keypoints
%p_i2(:,outFOV_idx) = [];

% append new landmarks in new frame
% Cj_P_hom_new = T_CjCi*[Ci_landmarks_new(1:3,:); ones(1,size(Ci_landmarks_new,2))];
% Cj_landmarks_updated = Cj_P_hom_new(1:3,:);

Cj_corresponding_inlier_landmarks = T_CjCi*[Ci_corresponding_inlier_landmarks(1:3,:); ones(1,size(Ci_corresponding_inlier_landmarks,2))];
Cj_corresponding_inlier_landmarks = Cj_corresponding_inlier_landmarks(1:3,:);
updated_keypoint_tracker = keypoints_tracker_old;


% % display statistics
% fprintf(['  Number of new landmarks triangulated: %i\n',...
%          '  Number of updated landmarks: %i\n'],...
%          size(Ci_landmarks_new,2), size(Cj_landmarks_updated,2));

end
