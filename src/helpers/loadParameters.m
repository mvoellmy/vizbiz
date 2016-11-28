function p = loadParameters(mode)
% Returns collection of parameters used throughout the VO pipeline
% 
% Input: point correspondences
%  - mode(scalar) : parameter setting mode
%
% Output:
%  - p(struct) : parameter struct


% initialization parameters
p.show_bootstrap_images = false;
p.init.use_KITTI_precalculated_init = true;

% continuous operation parameters
p.cont.run_on_first_ten_images = true;

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