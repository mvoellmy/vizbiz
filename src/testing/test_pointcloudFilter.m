
% randomly generate points
n_points = 1e4;
range = 80;
landmarks = range*rand(3,n_points) - range/2*ones(1,n_points);
landmarks_hom = [landmarks; ones(1,size(landmarks,2))];


figure('name','Filter test');

subplot(1,2,1);
plotLandmarks(landmarks, 'y', 'down');
%axis([-range range -range range -range range]);

% filter landmarks
cut_radius = 30;
[landmarks_hom_cut, ~] = applySphericalFilter(landmarks_hom, cut_radius);

subplot(1,2,2);
plotLandmarks(landmarks_hom_cut(1:3,:), 'y', 'down');
%axis([-range range -range range -range range]);
