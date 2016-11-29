function [R_WC_i, t_WC_i, keypoints_new, landmarks_updated] = ...
    processFrame(img_new,img_prev,keypoints_prev,landmarks,K,fig)
% TODO description
% 
% Input: point correspondences
%  - img_new(size) : current frame
%  - img_prev(size) : previous frame
%  - keypoints_prev(size) : ...
%  - K(3,3) : camera intrinsics matrix
%  - fig(-) : figure handle
%
% Output:
%  - R_WC_i(3,3) : ...
%  - t_WC_i(3,1) : ...
%  - keypoints_new(size) : ...
%  - landmarks_updated(size) : ...


% state propagation and pose estimation
[R_CW, t_CW, keypoints_new, ~,~,~] = ransacLocalization(...
    img_new, img_prev, keypoints_prev, landmarks, K);

if ~isempty(R_CW) && ~isempty(t_CW)
    disp('successfully localized');
else
    R_CW = eye(3,3);
    t_CW = zeros(3,1);
    warning('no transformation found');
end

% construct new camera pose
T_WC = [R_CW'   -R_CW'*t_CW;
        ones(1,3)         1];
R_WC_i = T_WC(1:3,1:3);
t_WC_i = T_WC(1:3,4);


% triangulation of new points with keypoint tracks
%M1 = K * eye(3,4);
%M2 = K * [R_CW, t_CW];
%landmarks_new = linearTriangulation(keypoints_prev,keypoints_new,M1,M2);
% landmarks_updated = [landmarks landmarks_new];
landmarks_updated = landmarks;


% update plots in fig


end
