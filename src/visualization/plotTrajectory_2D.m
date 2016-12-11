function plotTrajectory_2D (W_Pos_C)
% Plot projected 2D trajectory of camera.
% 
% Input:
%  - W_Pos_C(3xN) : camera positions in world frame
%
% Output: none

figure('name','Comparison against ground truth');
hold on;
plot(W_Pos_C(1,:),W_Pos_C(2,:),'b.-');
plot(W_Pos_C(1,1),W_Pos_C(2,1),'bsquare');
plot(W_Pos_C(1,end),W_Pos_C(2,end),'b*');
axis equal;

xlabel('x');
ylabel('y');
legend('visual odometry','start','end');

end