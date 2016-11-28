function p = load_parameters(mode)
% Detects k keypoints correspondeces
% Returns the sorted keypoints of both images
% 
% Input: point correspondences
%  - mode(scalar) : parameter setting mode
%
% Output:
%  - database_keypoints(k,3) : homogeneous keypoints of first image
%  - query_keypoints(k,3) : homogeneous keypoints of second image


% initialization parameters
p.show_bootstrap_img = false;
p.init.use_KITTI_precalculated_init = true;

% continuous operation parameters


% mode specific parameters
if mode == 1
    %TODO random values
    p.correspondences.lambda1 = 12;
    p.correspondences.lambda2 = 1;

    p.estimateEssential.mu2 = 13;
else
    error('invalid mode');
end


end