function [C2_landmarks_init, T_C1C2] = normalizeScale(C2_landmarks_init, T_C1C2, ground_truth, bootstrap_frame_idx_1, bootstrap_frame_idx_2)
% Normalize the scale such that it corresponds to the ground truth using
% the position vector
% 
% Input:
%  - C2_landmarks_init(3xN)     : list of 3D Points in C2
%  - T_C1C2(4X4)                : transformation matrix from C2 to C1
%  - W_ground_truth(2xM)        : ground thruth positions in world frame
%  - bootstrap_frame_1_idx(1x1) : dataset image index of img0
%  - bootstrap_frame_2_idx(1x1) : dataset image index of img1
% Output:
%  - C2_landmarks_init(3xN)     : list of 3D Points in C2
%  - T_C1C2(4X4)                : transformation matrix from C2 to C1

x_truth = ground_truth(bootstrap_frame_idx_2, 1) - ground_truth(bootstrap_frame_idx_1, 1);
z_truth = ground_truth(bootstrap_frame_idx_2, 2) - ground_truth(bootstrap_frame_idx_1, 2);

x_estimated = T_C1C2(1,4);
z_estimated = T_C1C2(3,4);

scale_factor = sqrt(x_truth^2 + z_truth^2)/sqrt(x_estimated^2 + z_estimated^2);

T_C1C2(1:3,4) = T_C1C2(1:3,4)*scale_factor;
C2_landmarks_init = C2_landmarks_init*scale_factor;

end