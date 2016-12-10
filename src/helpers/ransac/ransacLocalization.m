function [R_CiCj, Ci_t_CiCj, matched_query_keypoints_vu, matched_database_keypoints_vu, corresponding_matches, max_num_inliers_history] = ...
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
%  - corresponding_matches : todo ??
%  - max_num_inliers_history(1xnum_iterations) : number inlier history

global fig_cont fig_RANSAC_debug;

% find 2D correspondences % todo: move out of ransacLocalization()??
[matched_database_keypoints_vu,matched_query_keypoints_vu, corresponding_matches] = ...
    findCorrespondeces_cont(params,database_image,database_keypoints,query_image);
Ci_corresponding_landmarks = Ci_landmarks(:,corresponding_matches);

% display matched keypoints
if params.localization_ransac.show_matched_keypoints
    figure(fig_cont);
    subplot(1,2,1);
    plotPoints(matched_query_keypoints_vu,'g.');
    title('Current frame: Matched (green) keypoints');
    subplot(1,2,2);
    plotPoints(matched_query_keypoints_vu,'g.');
    title('Current frame: Matched (green) keypoints');
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
Cj_matched_query_keypoints_uv = flipud(matched_query_keypoints_vu);

% initialize RANSAC
best_guess_inliers = zeros(1, size(Cj_matched_query_keypoints_uv,2));
max_num_inliers_history = zeros(1,num_iterations);
max_num_inliers = 0;

% run RANSAC for pose estimation
for i = 1:num_iterations
    [landmark_sample,idx] = datasample(Ci_corresponding_landmarks,s,2,'Replace',false);
    keypoint_sample = Cj_matched_query_keypoints_uv(:,idx); % needed as [u,v]
    
    if params.localization_ransac.use_p3p
        normalized_bearings = K\[keypoint_sample; ones(1, 3)];
        for ii = 1:3
            normalized_bearings(:, ii) = normalized_bearings(:, ii) / norm(normalized_bearings(:, ii), 2);
        end
        poses = p3p(landmark_sample, normalized_bearings);
        R_C_W_guess = zeros(3, 3, 2);
        t_C_W_guess = zeros(3, 1, 2);
        for ii = 0:1
            R_W_C_ii = real(poses(:, (2+ii*4):(4+ii*4))); % rotation direction verified with description of p3p
            t_W_C_ii = real(poses(:, (1+ii*4)));
            R_C_W_guess(:,:,ii+1) = R_W_C_ii';
            t_C_W_guess(:,:,ii+1) = -R_W_C_ii'*t_W_C_ii;
        end
    else % no p3p
        M_C_W_guess = estimatePoseDLT(keypoint_sample',landmark_sample',K);
        R_C_W_guess = M_C_W_guess(:,1:3);
        t_C_W_guess = M_C_W_guess(:,end);
    end
    
    % count inliers
    Cj_projected_points_uv = projectPoints((R_C_W_guess(:,:,1)*Ci_corresponding_landmarks) +...
                                     repmat(t_C_W_guess(:,:,1),[1 size(Ci_corresponding_landmarks, 2)]),K);
    difference = Cj_matched_query_keypoints_uv - Cj_projected_points_uv;
    errors = sum(difference.^2,1);
    inliers = errors < params.localization_ransac.pixel_tolerance^2;
    
    if params.localization_ransac.use_p3p
        Cj_projected_points_uv = projectPoints((R_C_W_guess(:,:,2) * Ci_corresponding_landmarks) + ...
                                         repmat(t_C_W_guess(:,:,2),[1 size(Ci_corresponding_landmarks, 2)]),K);
        difference = Cj_matched_query_keypoints_uv - Cj_projected_points_uv;
        errors = sum(difference.^2, 1);
        alternative_is_inlier = errors < params.localization_ransac.pixel_tolerance^2;
        if nnz(alternative_is_inlier) > nnz(inliers)
            inliers = alternative_is_inlier;
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
    axis([0 num_iterations 0 size(Cj_matched_query_keypoints_uv,2)]);
    title('Max num inliers over iterations');
    
    % display fraction of inlier matches
    fprintf('  %0.2f %% of inliers matches found\n',100*max_num_inliers/size(Cj_matched_query_keypoints_uv,2));
end

% discard outliers
Cj_matched_query_keypoints_uv = Cj_matched_query_keypoints_uv(:, best_guess_inliers > 0);
matched_query_keypoints_vu = matched_query_keypoints_vu(:, best_guess_inliers > 0);
corresponding_matches = corresponding_matches(:, best_guess_inliers > 0);
matched_database_keypoints_vu = matched_database_keypoints_vu(:, corresponding_matches);
Ci_corresponding_landmarks = Ci_corresponding_landmarks(:, best_guess_inliers>0);

if (max_num_inliers == 0)
    R_CiCj = [];
    Ci_t_CiCj = [];
    fprintf('  No inlier matches found\n');
else
    % calculate [R,T] with best inlier points
    M_CjCi = estimatePoseDLT(...
        Cj_matched_query_keypoints_uv', ...
        Ci_corresponding_landmarks', K);
    R_CjCi = M_CjCi(:,1:3);
    Cj_t_CjCi = M_CjCi(:,end);
    
    % calculate inverse rotation matrices
    R_CiCj = R_CjCi';
    Ci_t_CiCj = -R_CiCj*Cj_t_CjCi;
end



% check for same number of query keypoints and database keypoints
assert(size(matched_query_keypoints_vu,2) == size(matched_database_keypoints_vu,2));

% display projected keypoints given best pose and inlier correspondences
if (nnz(best_guess_inliers) > 0 && params.localization_ransac.show_matched_keypoints)
    best_guess_projected_pts_uv = projectPoints((R_CiCj*Ci_corresponding_landmarks) + ...
                                                repmat(Ci_t_CiCj,[1 size(Ci_corresponding_landmarks, 2)]), K);
    figure(fig_cont);
    subplot(1,2,1);
    plotPoints(flipud(best_guess_projected_pts_uv),'yx');
    plotPoints(flipud(best_guess_projected_pts_uv),'yo');
    title('Current frame: Projected keypoints (yellow circles)');
end

% display inlier matches
if (nnz(best_guess_inliers) > 0 && params.localization_ransac.show_inlier_matches)
    figure(fig_cont);
    subplot(1,2,2);
    plotMatches(1:nnz(best_guess_inliers),matched_query_keypoints_vu,matched_database_keypoints_vu,'y-');
    title('Current frame: Inlier (yellow) matches found');
end

end
