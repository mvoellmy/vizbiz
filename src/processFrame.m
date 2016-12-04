function [R_WC_i, t_WC_i, p_new_matched, landmarks_updated] = ...
    processFrame(params,img_new,img_prev,keypoints_prev,landmarks,K)
% TODO description
% 
% Input:
%  - img_new(size) : current frame
%  - img_prev(size) : previous frame
%  - keypoints_prev(size) : ...
%  - landmarks(3xN) : 3D points
%  - K(3x3) : camera intrinsics matrix
%
% Output:
%  - R_WC_i(3x3) : ...
%  - t_WC_i(3x1) : ...
%  - keypoints_new(size) : ...
%  - landmarks_updated(size) : 3D points

global fig_cont;

% show current frame
if params.cont.show_current_image
    figure(fig_cont);
    imshow(img_new);
end

% state propagation and pose estimation
[R_CW,t_CW,p_new_matched,p_prev_matched,~,~] = ransacLocalization(params,img_new,img_prev,keypoints_prev,landmarks,K);

if ~isempty(R_CW) && ~isempty(t_CW)
    fprintf(' >> successfully localized\n');
else
    R_CW = eye(3,3);
    t_CW = zeros(3,1);
    fprintf('no transformation found\n');
end

% construct new camera pose
T_WC = [R_CW'   -R_CW'*t_CW;
        ones(1,3)         1];
R_WC_i = T_WC(1:3,1:3);
t_WC_i = T_WC(1:3,end);

% triangulation of new points with keypoint tracks
M1 = K * eye(3,4);
M2 = K * [R_CW, t_CW];
p_hom_prev_matched = [p_prev_matched;ones(1,size(p_prev_matched,2))];
p_hom_new_matched = [p_new_matched;ones(1,size(p_new_matched,2))];
landmarks_new = linearTriangulation(p_hom_prev_matched,p_hom_new_matched,M1,M2);

% append new landmarks
%landmarks_updated = [landmarks landmarks_new(1:3,:)];
landmarks_updated = landmarks_new;

end
