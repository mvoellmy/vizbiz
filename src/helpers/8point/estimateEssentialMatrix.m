function E = estimateEssentialMatrix(p1, p2, K1, K2)
% Estimates the essential matrix given matching point coordinates,
% and the camera calibration intrinsics.
%
% Input: point correspondences
%  - p1(3xN): homogeneous coordinates of 2-D points in image 1
%  - p2(3xN): homogeneous coordinates of 2-D points in image 2
%  - K1(3x3): calibration matrix of camera 1
%  - K2(3x3): calibration matrix of camera 2
%
% Output:
%  - E(3x3) : fundamental matrix

F = fundamentalEightPoint_normalized(p1,p2);

% compute the essential matrix from the fundamental matrix given K
E = K2'*F*K1;

end
