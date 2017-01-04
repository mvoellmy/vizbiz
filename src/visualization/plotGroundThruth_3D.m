function plotGroundThruth_3D (W_Pos_C, W_ground_truth)
% Plots 3D trajectory of camera.
% 
% Input:
%  - W_Pos_C(3xN) : camera positions in world frame
%  - W_ground_truth(2xM) : ground thruth positions in world frame
%
% Output: none

figure('name','Full 3D camera trajectory');
hold on;
plot(W_ground_truth(:,1),W_ground_truth(:,2),'k-');
plot3(W_Pos_C(1,:),W_Pos_C(2,:),W_Pos_C(3,:),'*', 'MarkerSize',20);
plot(W_ground_truth(1,1),W_ground_truth(1,2),'ksquare');
plot(W_ground_truth(end,1),W_ground_truth(end,2),'ko');
axis equal;

xlabel('x');
ylabel('y');
zlabel('z');
legend('ground truth','visual odometry','start','end');

end
