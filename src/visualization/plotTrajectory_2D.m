function plotTrajectory_2D (W_Pos_C, bootstrap_frame_idx_1, bootstrap_frame_idx_2)
% Plot projected 2D trajectory of camera.
% 
% Input:
%  - W_Pos_C(3xN) : camera positions in world frame
%  - bootstrap_frame_idx_1, 2: bootstrap frame index used to print the frame
%  numbers
%
% Output: none

figure('name','Comparison against ground truth');
hold on;
plot(W_Pos_C(1,:),W_Pos_C(2,:),'b.-','LineWidth',8);
plot(W_Pos_C(1,1),W_Pos_C(2,1),'bsquare');
plot(W_Pos_C(1,end),W_Pos_C(2,end),'b*');

axis equal;

% Print Frame numbers
%frame_id_txt = ['   ', num2str(bootstrap_frame_idx_1)];
%text(W_Pos_C(1,1), W_Pos_C(2,1), frame_id_txt, 'Color', 'green');
%for i=2:size(W_Pos_C, 3)
%    frame_id_txt = ['   ', num2str(i + bootstrap_frame_idx_2 - 2)];
%    text(W_Pos_C(1,i), W_Pos_C(2,i), frame_id_txt, 'Color', 'green');
%end

xlabel('x');
ylabel('y');
legend('visual odometry','start','end');

end
