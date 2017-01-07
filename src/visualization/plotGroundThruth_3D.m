function plotGroundThruth_3D (W_Pos_C, W_ground_truth)
% Plots 3D trajectory of camera.
% 
% Input:
%  - W_Pos_C(3xN) : camera positions in world frame
%  - W_ground_truth(2xM) : ground thruth positions in world frame
%
% Output: none

%figure('name','Full 3D camera trajectory');
clf
hold on;
plot(W_ground_truth(:,1),W_ground_truth(:,2),'k-');
plot3(W_Pos_C(1,:),W_Pos_C(2,:),W_Pos_C(3,:)+10,'b.-');
plot(W_ground_truth(1,1),W_ground_truth(1,2),'ksquare');
plot(W_ground_truth(end,1),W_ground_truth(end,2),'ko');

w = 50; % window size, todo: parametrize?
x_last = W_Pos_C(1,end);
y_last = W_Pos_C(2,end);

axis equal;
xlim([x_last-w/2 x_last+w/2])
ylim([y_last-w*1/3 y_last+w*2/3]);
axis vis3d;

xlabel('x');
ylabel('y');
zlabel('z');
%legend('ground truth','visual odometry','start','end');

hold off;

end
