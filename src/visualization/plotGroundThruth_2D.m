function plotGroundThruth_2D (W_Pos_C,W_ground_truth)
% Plot projected 2D trajectory of camera.
% 
% Input:
%  - W_Pos_C(3xN) : camera positions in world frame
%  - W_ground_truth(2xM) : ground thruth positions in world frame
%
% Output: none

figure('name','Comparison against ground truth');
hold on;
plot(W_ground_truth(1,:),W_ground_truth(2,:),'k-');
plot(W_Pos_C(1,:),W_Pos_C(2,:),'b.-');
plot(W_ground_truth(1,1),W_ground_truth(2,1),'ksquare');
plot(W_ground_truth(1,end),W_ground_truth(2,end),'ko');
plot(W_Pos_C(1,end),W_Pos_C(2,end),'b*');
axis equal;

xlabel('x');
ylabel('y');
legend('ground truth','visual odometry','start','end');

end