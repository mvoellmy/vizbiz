function [E, best_guess_inliers] = eightPointRansac(params, p_hom_i1, p_hom_i2, K1, K2)
% Estimates the essential matrix from a set of image keypoints and
% robustifies pose estimate with RANSAC rejecting outlier keypoint
% correspondences.
% 
% Input:
%  - params(struct) : parameter struct
%  - p_hom_i1(3xN) : homogeneous 2D points of image 1, each [u v 1]
%  - p_hom_i2(3xN) : homogeneous 2D points of image 2, each [u v 1]
%  - K1(3x3) : intrinsics matrix of camera 1
%  - K2(3x3) : intrinsics matrix of camera 2
%
% Output:
%  - E(3x3) : essential matrix
%  - best_guess_inliers(todo) : inlier indices of kp matches satisfying E

% sample size
s = 8;

% needed iterations to reach delta-probable outlier free solution
% num_iterations = ceil(log(1-params.eightPoint_ransac.p_success)/...
%                       log(1-params.eightPoint_ransac.fract_inliers^s));

num_iterations = 3000;
% initialize RANSAC
best_guess_inliers = NaN(1,size(p_hom_i2,2));
max_num_inliers_history = NaN(1,num_iterations);
max_num_inliers = 0;
inliers = zeros(1,size(p_hom_i1,2));

% run RANSAC for pose estimation
for i=1:num_iterations
    [p_hom_i1_sample, idx] = datasample(p_hom_i1, s, 2, 'Replace', false);
    p_hom_i2_sample = p_hom_i2(:,idx);
    
    % estimate fundamental matrix given data sample
    F_guess = fundamentalEightPoint_normalized(p_hom_i1_sample, p_hom_i2_sample);
    
    % count inliers based on pixel errors
    errors = geomError2EpipolarLine(p_hom_i1, p_hom_i2, F_guess);
    inliers = errors <= params.eightPoint_ransac.max_error;
    num_inliers = nnz(inliers);

    if (num_inliers > max_num_inliers)        
        % update benchmark
        max_num_inliers = num_inliers;
        max_num_inliers_history(i) = num_inliers;        
        
        % save best inliers
        best_guess_inliers = inliers;
    else
        max_num_inliers_history(i) = max_num_inliers;
    end
end

% display fraction of inlier matches
updateConsole(params,...
              sprintf('  %0.2f perc. of inliers matches found\n',...
              100*max_num_inliers/size(p_hom_i1,2)));

% display count of inliers evolution
if params.eightPoint_ransac.show_iterations
    figure('name','8-point estimation RANSAC');
    plot(max_num_inliers_history);
    axis([0 num_iterations 0 size(p_hom_i1,2)]);
    title('Max num inliers over iterations');    
end

% rerun on inlier correspondences
best_guess = fundamentalEightPoint_normalized(p_hom_i1(:,best_guess_inliers),...
                                              p_hom_i2(:,best_guess_inliers));

% compute the essential matrix from the fundamental matrix given K
E = K2'*best_guess*K1;

if isnan(E)
    updateConsole(params, '  No inlier matches found\n');
end

end
