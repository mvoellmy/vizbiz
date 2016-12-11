function [R_C_W, t_C_W, matched_query_keypoints, matched_database_keypoints, max_num_inliers_history] = ransacLocalization(params, ...
    query_image, database_image, database_keypoints, p_W_landmarks, K)
% TODO description
% query_keypoints should be 2x1000
% all_matches should be 1x1000 and correspond to the output from the
%   matchDescriptors() function from exercise 3.
% inlier_mask should be 1xnum_matched (!!!) and contain, only for the
%   matched keypoints (!!!), 0 if the match is an outlier, 1 otherwise.
%
% Inputs:
%  - params: Parameter struct
%  - Query_image: New image. We search its relative rotation matrix to database
%    image
%  - database_image: Last image. It defines the current world frame.
%  - database_keypoints (2xN): Pixel coordinates of keypoints
%  - p_w_landmarks(3xN) : 3D points of database image
%  - K : Calibration matrix of the camera (assumed to be the same for both
%    images)
%
% Output:
%  - t_C_W(3x1) : translation vector
%  - R_C_W(3x3) : rotation matrix camera 1 (world frame) to camera 2
%  - matched_query_keypoints() : [v,u]
%  - matched_database_keypoints : [v,u]

global fig_cont;
global fig_RANSAC_debug;

% find 2D correspondences
[query_keypoints,matches] = findCorrespondeces_cont(params,database_image,database_keypoints,query_image);
matched_query_keypoints = query_keypoints(:,matches>0);
matched_database_keypoints = database_keypoints(:,matches(matches > 0));
corresponding_landmarks  = p_W_landmarks(:,matches(matches > 0));

% check for same number of query keypoints and database keypoints
assert(size(matched_query_keypoints,2) == size(matched_database_keypoints,2));
%corresponding_landmarks = p_W_landmarks(:,);


% display matched keypoints
if params.localization_ransac.show_matched_keypoints
    figure(fig_cont);
    plotPoints(matched_query_keypoints,'g.');
    title('Current frame: Matched query keypoints');
end

% choose RANSAC options
if params.localization_ransac.use_p3p
    s = 3;
    num_iterations = params.localization_ransac.num_iterations_pnp;
else
    s = 6;
    num_iterations = params.localization_ransac.num_iterations_DLT;
end

% initialize RANSAC
best_guess_inliers = zeros(1, size(matched_query_keypoints,2));
matched_query_keypoints = flipud(matched_query_keypoints); % !!
max_num_inliers_history = zeros(1,num_iterations);
max_num_inliers = 0;

% run RANSAC for pose estimation
for i = 1:num_iterations
    [landmark_sample, idx] = datasample(...
        corresponding_landmarks, s, 2, 'Replace', false);
    keypoint_sample = matched_query_keypoints(:, idx); % needed as [u,v]
    
    if params.localization_ransac.use_p3p
        normalized_bearings = K\[keypoint_sample; ones(1, 3)];
        for ii = 1:3
            normalized_bearings(:, ii) = normalized_bearings(:, ii) / ...
                norm(normalized_bearings(:, ii), 2);
        end
        poses = p3p(landmark_sample, normalized_bearings);
        R_C_W_guess = zeros(3, 3, 2);
        t_C_W_guess = zeros(3, 1, 2);
        for ii = 0:1
            R_W_C_ii = real(poses(:, (2+ii*4):(4+ii*4)));
            t_W_C_ii = real(poses(:, (1+ii*4)));
            R_C_W_guess(:,:,ii+1) = R_W_C_ii';
            t_C_W_guess(:,:,ii+1) = -R_W_C_ii'*t_W_C_ii;
        end
    else
        M_C_W_guess = estimatePoseDLT(keypoint_sample',landmark_sample',K);
        R_C_W_guess = M_C_W_guess(:,1:3);
        t_C_W_guess = M_C_W_guess(:,end);
    end
    
    % count inliers
    projected_points = projectPoints(...
        (R_C_W_guess(:,:,1)*corresponding_landmarks) + repmat(t_C_W_guess(:,:,1),[1 size(corresponding_landmarks, 2)]), K);
    difference = matched_query_keypoints - projected_points;
    errors = sum(difference.^2, 1);
    inliers = errors < params.localization_ransac.pixel_tolerance^2;
    
    if params.localization_ransac.use_p3p
        projected_points = projectPoints(...
            (R_C_W_guess(:,:,2) * corresponding_landmarks) + ...
            repmat(t_C_W_guess(:,:,2), ...
            [1 size(corresponding_landmarks, 2)]), K);
        difference = matched_query_keypoints - projected_points;
        errors = sum(difference.^2, 1);
        alternative_is_inlier = errors < params.localization_ransac.pixel_tolerance^2;
        if nnz(alternative_is_inlier) > nnz(inliers)
            inliers = alternative_is_inlier;
        end
    end
    
    % save new model if better then old one
    if nnz(inliers) > max_num_inliers && nnz(inliers) >= 6
        max_num_inliers = nnz(inliers);        
        best_guess_inliers = inliers;
    end
    
    max_num_inliers_history(i) = max_num_inliers;
end


% display count of inliers evolution
if params.localization_ransac.show_iterations
    figure(fig_RANSAC_debug);
    plot(max_num_inliers_history);
    axis([0 num_iterations 0 size(matched_query_keypoints,2)]);
    title('Max num inliers over iterations');
    
    % display fraction of inlier matches
    fprintf('  %0.2f %% of inliers matches found\n',100*max_num_inliers/size(matched_query_keypoints,2));
end

% discard outliers
matched_query_keypoints = matched_query_keypoints(:, best_guess_inliers > 0);
%corresponding_matches = corresponding_matches(:, best_guess_inliers > 0);
matched_database_keypoints = matched_database_keypoints(:, best_guess_inliers > 0);

if (max_num_inliers == 0)
    R_C_W = [];
    t_C_W = [];
    fprintf('no inlier matches found\n');
else % calculate [R,T] with best inlier points
    M_C_W = estimatePoseDLT(...
        matched_query_keypoints', ...
        corresponding_landmarks', K);
    R_C_W = M_C_W(:,1:3);
    t_C_W = M_C_W(:,end);
end


% this is already done in findCorespondences ??
%matched_database_keypoints = matched_database_keypoints(:, corresponding_matches);




% display projected keypoints given best pose and inlier corespondences
if params.localization_ransac.show_matched_keypoints
    figure(fig_cont);
    
    best_guess_projected_pts = projectPoints(...
        (R_C_W*corresponding_landmarks) + repmat(t_C_W,[1 size(corresponding_landmarks, 2)]), K);
    plotPoints(flipud(best_guess_projected_pts),'yx');
    viscircles(best_guess_projected_pts', params.localization_ransac.pixel_tolerance * ones(size(best_guess_projected_pts,2),1),'LineStyle','-', 'color', 'y');
end

display inlier matches
if (nnz(best_guess_inliers) > 0 && params.localization_ransac.show_inlier_matches)
    figure(fig_cont);
%     plotMatches(1:nnz(best_guess_inliers),matched_query_keypoints,matched_database_keypoints,'y-');
    plot([matched_query_keypoints(2,:); matched_database_keypoints(2,:)], [matched_query_keypoints(1,:); matched_database_keypoints(1,:)], 'y-', 'Linewidth', 1);
    % plotMatches(1:nnz(best_guess_inliers),p_hom_i2(1:2,best_guess_inliers),p_hom_i1(1:2,best_guess_inliers),'y-');
    title('Current frame: Inlier matches found');
end

% flip keypointsbest_guess_inliers
matched_query_keypoints = flipud(matched_query_keypoints);
matched_database_keypoints = flipud(matched_database_keypoints);

end
