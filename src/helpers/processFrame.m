function [T_CiCj, p_new_matched, Cj_landmarks_updated] = processFrame(params,img_new,img_prev,keypoints_prev,Ci_landmarks_prev,K)
% TODO description
% 
% Input:
%  - params(struct) : parameter struct
%  - img_new(size) : current frame
%  - img_prev(size) : previous frame
%  - keypoints_prev(2xN) : 2D points, [v u]
%  - Ci_landmarks_prev(3xN) : 3D points
%  - K(3x3) : camera intrinsics matrix
%
% Output:
%  - T_CiCj(4x4) : transformation Cj to Ci
%  - p_new_matched(2xN) : newly matched keypoints, [v u] 
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

% TODO: keypoint tracks 

% state propagation and pose estimation
[R_CiCj,Ci_t_CiCj,p_new_matched,p_prev_matched,~,~] = ransacLocalization(params,img_new,img_prev,keypoints_prev,Ci_landmarks_prev,K);

if (~isempty(R_CiCj) && ~isempty(Ci_t_CiCj))
    updateConsole(params, '  >> Successfully localized\n');
else
    R_CiCj = eye(3,3);
    Ci_t_CiCj = zeros(3,1);
    updateConsole(params, '  No transformation found\n');
end

% construct new camera pose
T_CiCj = [R_CiCj   Ci_t_CiCj;
          zeros(1,3)       1];
      
T_CjCi = [R_CiCj'   -R_CiCj'*Ci_t_CiCj;
          zeros(1,3)                 1];

% triangulate new points with keypoint
Mi = K * eye(3,4);
Mj = K * [R_CiCj, Ci_t_CiCj];
p_hom_prev_matched = [p_prev_matched; ones(1,size(p_prev_matched,2))];
p_hom_new_matched = [p_new_matched; ones(1,size(p_new_matched,2))];
Ci_landmarks_new = linearTriangulation(p_hom_prev_matched,p_hom_new_matched,Mi,Mj);

% discard landmarks not contained in cylindrical neighborhood
[Ci_landmarks_new, outFOV_idx] = applyCylindricalFilter(Ci_landmarks_new, params.cont.landmarks_cutoff);

% todo: remove corresponding keypoints
%p_i2(:,outFOV_idx) = [];

% append new landmarks in new frame
Cj_P_hom_new = T_CjCi*[Ci_landmarks_new(1:3,:); ones(1,size(Ci_landmarks_new,2))];
Cj_landmarks_updated = Cj_P_hom_new(1:3,:);
          
% display statistics
updateConsole(params,...
    sprintf(['  Number of new landmarks triangulated: %i\n',...
             '  Number of updated landmarks: %i\n'],...
             size(Ci_landmarks_new,2), size(Cj_landmarks_updated,2)));

end
