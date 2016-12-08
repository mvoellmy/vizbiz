function [T_C1C2_i, p_new_matched, C2_landmarks_updated] = ...
    processFrame(params,img_new,img_prev,keypoints_prev,C1_landmarks,K)
% TODO description
% 
% Input:
%  - params(struct) : parameter struct
%  - img_new(size) : current frame
%  - img_prev(size) : previous frame
%  - keypoints_prev(size) : 2D points , each [v u]
%  - C1_landmarks(3xN) : 3D points
%  - K(3x3) : camera intrinsics matrix
%
% Output:
%  - T_C1C2_i(4x4) : ...
%  - p_new_matched(size) : 
%  - C2_landmarks_updated(size) : 3D points

global fig_cont;

% show current frame
if params.cont.show_current_image
    figure(fig_cont);
    imshow(img_new);
end

% state propagation and pose estimation
[R_CW,t_CW,p_new_matched,p_prev_matched,~,~] = ransacLocalization(params,img_new,img_prev,keypoints_prev,C1_landmarks,K);

if ~isempty(R_CW) && ~isempty(t_CW)
    fprintf(' >> Successfully localized\n');
else
    R_CW = eye(3,3);
    t_CW = zeros(3,1);
    fprintf('No transformation found\n');
end

% construct new camera pose
T_C1C2_i = [R_CW'   -R_CW'*t_CW;
            ones(1,3)         1];

% triangulate new points with keypoint tracks % TODO
M1 = K * eye(3,4);
M2 = K * [R_CW, t_CW];
p_hom_prev_matched = [p_prev_matched;ones(1,size(p_prev_matched,2))];
p_hom_new_matched = [p_new_matched;ones(1,size(p_new_matched,2))];
landmarks_new = linearTriangulation(p_hom_prev_matched,p_hom_new_matched,M1,M2); % TODO assure good keypoints

% append new landmarks
C2_landmarks_updated = [C1_landmarks landmarks_new(1:3,:)];

% display statistics
fprintf(['  Number of new landmarks triangulated: %i\n',...
         '  Total number of landmarks: %i\n\n'],...
         size(landmarks_new,2), size(C2_landmarks_updated,2));

end
