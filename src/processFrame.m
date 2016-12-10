function [T_CiCj, p_new_matched, Cj_landmarks_updated] = processFrame(params,img_new,img_prev,keypoints_prev,Ci_landmarks,K)
% TODO description
% 
% Input:
%  - params(struct) : parameter struct
%  - img_new(size) : current frame
%  - img_prev(size) : previous frame
%  - keypoints_prev(2xN) : 2D points,[v u]
%  - C1_landmarks(3xN) : 3D points
%  - K(3x3) : camera intrinsics matrix
%
% Output:
%  - T_CiCj(4x4) : transformation of Cj relative to Ci
%  - p_new_matched(2xN) : newly matched keypoints, [v u] 
%  - Cj_landmarks_updated(3xN) : 3D points in frame Cj

global fig_cont;

% show current frame
if params.cont.show_current_image
    figure(fig_cont);
    subplot(1,2,1);
    imshow(img_new);
    subplot(1,2,2);
    imshow(img_new);
end

% state propagation and pose estimation
[R_CiCj,Ci_t_CiCj,p_new_matched,p_prev_matched,~,~] = ransacLocalization(params,img_new,img_prev,keypoints_prev,Ci_landmarks,K);

if (~isempty(R_CiCj) && ~isempty(Ci_t_CiCj))
    fprintf(' >> Successfully localized\n');
else
    R_CiCj = eye(3,3);
    Ci_t_CiCj = zeros(3,1);
    fprintf('  No transformation found\n');
end

% construct new camera pose
T_CiCj = [R_CiCj   Ci_t_CiCj;
          ones(1,3)       1];

% triangulate new points with keypoint tracks % TODO
M1 = K * eye(3,4);
M2 = K * [R_CiCj, Ci_t_CiCj];
p_hom_prev_matched = [p_prev_matched;ones(1,size(p_prev_matched,2))];
p_hom_new_matched = [p_new_matched;ones(1,size(p_new_matched,2))];
Ci_landmarks_new = linearTriangulation(p_hom_prev_matched,p_hom_new_matched,M1,M2);

% check for landmarks with negative Z component in Cj frame
% TODO assure good landmarks
% any(Ci_landmarks_new(3,:)<0)

% append new landmarks in new frame
%Cj_landmarks_updated = [Ci_landmarks Ci_landmarks_new(1:3,:)];
Cj_landmarks_updated = T_CiCj(1:3,1:3)'*[Ci_landmarks Ci_landmarks_new(1:3,:)];

% display statistics
fprintf(['  Number of new landmarks triangulated: %i\n',...
         '  Total number of landmarks: %i\n\n'],...
         size(Ci_landmarks_new,2), size(Cj_landmarks_updated,2));

end
