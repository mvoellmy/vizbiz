function [R_CjCi, Ci_t_CiCj, matched_query_keypoints, matched_database_keypoints, corr_ldk_matches, max_num_inliers_history] = ...
    ransacLocalization(params, query_image, database_image, database_keypoints, Ci_landmarks, K)
% TODO description
%
% Inputs:
%  - params(struct) : parameter struct
%  - query_image(size) : new image
%  - database_image(size) : previous image, it defines Ci frame
%  - database_keypoints(2xN): previous keypoints, [v u]
%  - Ci_landmarks(3xN) : 3D points in previous camera frame Ci
%  - K(3x3) : camera intrinsics matrix
%
% Output:
%  - R_CiCj(3x3) : rotation matrix Cj to Ci
%  - Ci_t_CiCj(3x1) : translation vector of Ci to Cj expressed in frame Ci
%  - matched_query_keypoints(2xN) : re-matched query keypoints, [v u]
%  - matched_database_keypoints(2xN) : re-matched database keypoints, [v u]
%  - corr_ldk_matches : todo ??
%  - max_num_inliers_history(1xnum_iterations) : number inlier history

global fig_cont fig_RANSAC_debug;

% find 2D correspondences % todo: move out of ransacLocalization()??
[matched_database_keypoints,matched_query_keypoints,corr_ldk_matches] = ...
    findCorrespondeces_cont(params,database_image,database_keypoints,query_image);
Ci_corresponding_landmarks = Ci_landmarks(:,corr_ldk_matches);

% display matched keypoints
if params.localization_ransac.show_matched_keypoints
    figure(fig_cont);
    subplot(2,1,1);
    plotPoints(matched_query_keypoints,'g.');
    plotCircles(matched_query_keypoints,'y',params.localization_ransac.pixel_tolerance);
    title('Matched (green) keypoints with (yellow) confidence');
    subplot(2,1,2);
    plotPoints(matched_query_keypoints,'g.');
    title('Matched (green) keypoints');
end

% choose RANSAC options
if params.localization_ransac.use_p3p
    s = 3;
    num_iterations = params.localization_ransac.num_iterations_pnp;
else
    s = 6;
    num_iterations = params.localization_ransac.num_iterations_DLT;
end

% flip query keypoints for error estimation with projected_points
matched_query_keypoints = flipud(matched_query_keypoints);

% initialize RANSAC
best_guess_inliers = zeros(1, size(matched_query_keypoints,2));
max_num_inliers_history = zeros(1,num_iterations);
max_num_inliers = 0;

% run RANSAC for pose estimation
for i = 1:num_iterations
    [landmark_sample,idx] = datasample(Ci_corresponding_landmarks,s,2,'Replace',false);
    keypoint_sample = matched_query_keypoints(:,idx); % needed as [u,v]
    
    if ~params.localization_ransac.use_p3p % todo: needed?
        fprintf('Current datasample index of Cj_matched_query_keypoint_uv: %d, %d, %d, %d, %d, %d\n',idx(1),idx(2),idx(3),idx(4),idx(5),idx(6));
    end
    
    if params.localization_ransac.use_p3p
        normalized_bearings = K\[keypoint_sample; ones(1, 3)];
        for ii = 1:3
            normalized_bearings(:, ii) = normalized_bearings(:, ii) / norm(normalized_bearings(:, ii), 2);
        end
        poses = p3p(landmark_sample, normalized_bearings);
        R_CjCi_guess = zeros(3, 3, 2);
        Cj_t_CjCi_guess = zeros(3, 1, 2); % todo: correct base frame?
        for ii = 0:1
            R_W_C_ii = real(poses(:, (2+ii*4):(4+ii*4))); % rotation direction verified with description of p3p
            t_W_C_ii = real(poses(:, (1+ii*4))); % expressed in W
            R_CjCi_guess(:,:,ii+1) = R_W_C_ii';
            Cj_t_CjCi_guess(:,:,ii+1) = -R_W_C_ii'*t_W_C_ii;
        end
    else
        M_C_W_guess = estimatePoseDLT(keypoint_sample',landmark_sample',K);
        R_CjCi_guess = M_C_W_guess(:,1:3);
        Cj_t_CjCi_guess = M_C_W_guess(:,end);
    end
    
    % count inliers
    projected_points = projectPoints((R_CjCi_guess(:,:,1)*Ci_corresponding_landmarks) +...
                                     repmat(Cj_t_CjCi_guess(:,:,1),[1 size(Ci_corresponding_landmarks, 2)]),K);
    difference = matched_query_keypoints - projected_points;
    errors = sum(difference.^2,1);
    inliers = errors < params.localization_ransac.pixel_tolerance^2;
    
    if params.localization_ransac.use_p3p
        projected_points = projectPoints((R_CjCi_guess(:,:,2) * Ci_corresponding_landmarks) +...
                                         repmat(Cj_t_CjCi_guess(:,:,2),[1 size(Ci_corresponding_landmarks, 2)]),K);
        difference = matched_query_keypoints - projected_points;
        errors = sum(difference.^2, 1);
        alternative_inliers = errors < params.localization_ransac.pixel_tolerance^2;
        if nnz(alternative_inliers) > nnz(inliers)
            inliers = alternative_inliers;
        end
    end
    
    % save new model if better then old one
    if (nnz(inliers) > max_num_inliers && nnz(inliers) >= 6)
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
    fprintf('  Max number of inlier matches found: %i (%0.2f %%)\n',...
            max_num_inliers,100*max_num_inliers/size(matched_query_keypoints,2));
end

% discard outliers
matched_query_keypoints = matched_query_keypoints(:, best_guess_inliers);
matched_database_keypoints = matched_database_keypoints(:, best_guess_inliers);

corr_ldk_matches_inliers = corr_ldk_matches(:, best_guess_inliers);
Ci_corresponding_landmarks = Ci_landmarks(:, corr_ldk_matches_inliers);

if (max_num_inliers == 0)
    R_CjCi = [];
    Cj_t_CjCi = [];
    fprintf('  No inlier matches found\n');
else
    % calculate [R,T] with best inlier points
    M_CjCi = estimatePoseDLT(...
        matched_query_keypoints', ...
        Ci_corresponding_landmarks', K);
    R_CjCi = M_CjCi(:,1:3);
    Cj_t_CjCi = M_CjCi(:,end);
end

% calculate inverse rotation matrices
R_CiCj = R_CjCi';
Ci_t_CiCj = -R_CiCj*Cj_t_CjCi;

% check for same number of query keypoints and database keypoints
assert(size(matched_query_keypoints,2) == size(matched_database_keypoints,2));

% display projected keypoints given best pose and inlier correspondences
if (max_num_inliers > 0 && params.localization_ransac.show_matched_keypoints)
    best_guess_projected_pts = projectPoints((R_CjCi*Ci_corresponding_landmarks) + ...
                                                repmat(Cj_t_CjCi,[1 size(Ci_corresponding_landmarks, 2)]), K);
    figure(fig_cont);
    subplot(2,1,1);
    plotPoints(flipud(best_guess_projected_pts),'yx');
    title('Projected keypoints (yellow crosses)');
end

% flip keypoints back to restore [v u] order
matched_query_keypoints = flipud(matched_query_keypoints);

% display inlier matches
if (max_num_inliers > 0 && params.localization_ransac.show_inlier_matches)
    figure(fig_cont);
    subplot(2,1,2);
    plotMatches(1:nnz(best_guess_inliers),matched_query_keypoints,matched_database_keypoints,'y-');
    title('Inlier (yellow) matches found');
end

end
