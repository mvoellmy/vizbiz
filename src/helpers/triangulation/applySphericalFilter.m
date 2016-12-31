function [C_P_hom, outFOV_idx] = applySphericalFilter(params, C_P_hom, cutoff_radius)
% Removes landmarks which are more than cutoff distance away in of z
% direction and behind camera (negative z distances).
%
% Input:
%  - params(struct) : parameter struct
%  - P_hom(4xN) : list of homogeneous 3D Points in camera frame with y 
% pointing vertically down
%  - cutoff_radius(1x1) : constant radial distance
%
% Output:
%  - P_hom(4x(N-O)) : filtered homogeneous 3D Points
%  - outFOV_idx(Ox1) : indeces of the Outliers. Can be used to remove the
%  corresponding 2D keypoints.
%
% Definitions:
% - O(int) : Number of Outliers

% todo: change indices once newest best_rot is merged with this ???
size_unfiltered_landmarks = size(C_P_hom, 2);

outFOV_idx = find( ( (C_P_hom(3,:) <0) | (sqrt(C_P_hom(1,:).^2 + C_P_hom(2,:).^2+ C_P_hom(3,:).^2) > cutoff_radius) ) );
% Note: Single | is used to compare vectors. 

C_P_hom(:,outFOV_idx) = [];
size_filtered_landmarks = size(C_P_hom, 2);

% display filter statistic
updateConsole(params,...
              sprintf('  %0.2f perc. (%i/%i) of landmarks were accepted\n',...
              100*size_filtered_landmarks/size_unfiltered_landmarks, size_filtered_landmarks, size_unfiltered_landmarks));

end
