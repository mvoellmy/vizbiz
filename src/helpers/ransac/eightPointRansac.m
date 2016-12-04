function E = eightPointRansac(params, p_hom_i1, p_hom_i2, K1, K2)
% TODO
% 
% Input:
%  - params(struct) : parameter struct
%  - p_hom_i1(3xN) : homogeneous 2D points of image 1
%  - p_hom_i2(3xN) : homogeneous 2D points of image 2
%  - K1(3x3) : intrinsics matrix of camera 1
%  - K2(3x3) : intrinsics matrix of camera 2
%
% Output:
%  - best_guess_history(3x3xnum_iterations) : model history
%  - max_num_inliers_history(1xnum_iterations) : number inlier history

global fig_init;

% sample size
s = 8;

% needed iterations to reach delta-probable outlier free solution
num_iterations = ceil(log(1-params.eightPoint_ransac.p_success)/...
                 log(1-params.eightPoint_ransac.fract_inliers^s));

best_guess = NaN(3,3);          
best_guess_history = NaN(3,3,num_iterations);
max_num_inliers_history = NaN(1,num_iterations);
max_num_inliers = 0;
inliers = zeros(1,size(p_hom_i1,2));

for i=1:num_iterations
    p_hom_i1_sample = datasample(p_hom_i1,s,2,'Replace',false);
    p_hom_i2_sample = datasample(p_hom_i2,s,2,'Replace',false);

    % estimate fundamental matrix given data sample
    F_guess = fundamentalEightPoint_normalized(p_hom_i1_sample,p_hom_i2_sample);
    
    % count how many other points fit this guess based on 
    % squared epipolar-line-to-point distance    
    errors = squaredEpipolarLineDistance(K1\p_hom_i1,K2\p_hom_i2,F_guess);
    inliers = errors <= params.eightPoint_ransac.max_error; % pixel error
    num_inliers = nnz(inliers);

    if (num_inliers > max_num_inliers)
        % rerun on all point correspondences
        F_guess = fundamentalEightPoint_normalized(p_hom_i1,p_hom_i2);
        
        % update benchmark
        max_num_inliers = num_inliers;
        best_guess = F_guess;
        best_guess_history(:,:,i) = best_guess;
        max_num_inliers_history(i) = num_inliers;
    else
        best_guess_history(:,:,i) = best_guess;
        max_num_inliers_history(i) = max_num_inliers;
    end
end

% display count of inliers evolution
if params.eightPoint_ransac.show_iterations
    figure('name','8-point estimation RANSAC');
    plot(max_num_inliers_history);
    axis([0 num_iterations 0 size(p_hom_i1,2)]);
    title('Max num inliers over iterations');
end

% display inlier matches
if params.eightPoint_ransac.show_inlier_matches
    figure(fig_init);
    h = plotMatches(1:nnz(inliers),p_hom_i2(1:2,inliers),p_hom_i1(1:2,inliers),'g-');
    legend(h, 'inlier matches');
    title('Inlier matches found');
end

% compute the essential matrix from the fundamental matrix given K
E = K2'*best_guess*K1;
    
end
