function [P_hom, outFOV_idx] = applyCylindricalFilter(P_hom, cutoff_constant)
% Remove landmarks which are more than cutoff_constant*median of z away from the
% camera.
% Median of z is used since the points are naturally the
% farthest away in z.
%
% Input:
%  - P_hom(4xN) : list of 3D Points in Worldframe with y pointing
%  vertically down
%  - cutoff_constant(int) : constant which is multiplied by the median of the landmarks in z
%
% Output:
%  - P_hom(4x(N-O)) : filtered 3D Points
%  - outFOV_idx(Ox1) : indeces of the Outliers. Can be used to remove the
%  corresponding 2D keypoints.
%
% Definitions:
% - O(int) : Number of Outliers

% todo: change indices once newest best_rot is merged with this
size_unfiltered_landmarks = size(P_hom, 2);

cutoff_radius = cutoff_constant * median(P_hom(3,:));

outFOV_idx = find( ( (P_hom(3,:) <0) | (sqrt(P_hom(1,:).^2 + P_hom(3,:).^2) > cutoff_radius ) ));
% Note: Single | is used to compare vectors. 

P_hom(:,outFOV_idx) = [];
size_filtered_landmarks = size(P_hom, 2);

fprintf('  %0.2f%% (%i/%i) of Landmarks were accepted\n',...
        100*size_filtered_landmarks/size_unfiltered_landmarks, size_filtered_landmarks, size_unfiltered_landmarks);

end
