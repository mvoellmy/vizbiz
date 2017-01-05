function plotTrajectoryVsGT_2D (W_Pos_C,W_ground_truth, bootstrap_frame_idx_1, bootstrap_frame_idx_2)
% Plot projected 2D trajectory of camera versus ground truth data.
% 
% Input:
%  - W_Pos_C(3xN) : camera positions in world frame
%  - W_ground_truth(2xM) : ground thruth positions in world frame
%  - bootstrap_frame_idx_1, 2: bootstrap frame index used to print the frame
%  numbers
%
% Output: none

plotTrajectory_2D (W_Pos_C, bootstrap_frame_idx_1, bootstrap_frame_idx_2);
plot(W_ground_truth(1,:),W_ground_truth(2,:),'k-');
legend('ground truth','visual odometry','start','end');

end