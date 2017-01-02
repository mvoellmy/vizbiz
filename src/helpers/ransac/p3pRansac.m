function [R_CiCj, Ci_t_CiCj, matched_query_inlier_keypoints, Ci_corresponding_inlier_landmarks, best_guess_inliers] = ...
    p3pRansac(params, matched_query_keypoints, Ci_corresponding_landmarks, K)
% Todo: description
%
% Inputs:
%  - params(struct) : parameter struct
%  - matched_query_keypoints (2xN) : sorted matched query keypoints that
%    have a correspondent landmark [v,u]
%  - Ci_corresponding_landmarks (3xN) : 3D points in previous camera frame Ci to
%    database_keypoints (same index)
%  - K(3x3) : camera intrinsics matrix
%
% Output:
%  - R_CiCj(3x3) : rotation matrix Cj to Ci
%  - Ci_t_CiCj(3x1) : translation vector of Ci to Cj expressed in frame Ci
%  - matched_query_inlier_keypoints(2xN) : inlier query keypoints, [v u]
%  - Ci_corresponding_inlier_landmarks(3xN) : inlier landmarks
%  - best_guess_inliers(todo) : inlier indices of kp matches satisfying
%  [R,t]

global fig_cont fig_RANSAC_debug gui_handles;

% choose RANSAC options
if params.localization_ransac.use_p3p
    s = 3;
    num_iterations = params.localization_ransac.num_iterations_pnp; % todo: calculate num_iterations, or fix?
else
    s = 6;
    num_iterations = params.localization_ransac.num_iterations_DLT;
end

% flip query keypoints for error estimation with projected_points
matched_query_keypoints_uv = flipud(matched_query_keypoints);

% initialize RANSAC
best_guess_inliers = zeros(1,size(matched_query_keypoints_uv,2));
max_num_inliers_history = zeros(1,num_iterations);
max_num_inliers = 0;

% run RANSAC for pose estimation
R_CjCi_best_guess = zeros(3,3);
Cj_t_CjCi_best_guess = zeros(3,1);

for i = 1:num_iterations
    [landmark_sample, idx] = datasample(Ci_corresponding_landmarks, s, 2, 'Replace', false);
    keypoint_sample = matched_query_keypoints_uv(:,idx); % needed as [u,v]
    
    if ~params.localization_ransac.use_p3p % todo: needed?
        fprintf('Current datasample index of Cj_matched_query_keypoint_uv: %d, %d, %d, %d, %d, %d\n',idx(1),idx(2),idx(3),idx(4),idx(5),idx(6));
    end
    
    if params.localization_ransac.use_p3p
        normalized_bearings = K\[keypoint_sample; ones(1,3)];
        for ii = 1:3
            normalized_bearings(:,ii) = normalized_bearings(:,ii) / norm(normalized_bearings(:,ii), 2);
        end
        poses = p3p(landmark_sample, normalized_bearings);
        R_CjCi_guess = zeros(3,3,2);
        Cj_t_CjCi_guess = zeros(3,1,2);
        for ii = 0:1
            R_W_C_ii = real(poses(:,(2+ii*4):(4+ii*4))); % rotation direction verified with description of p3p
            t_W_C_ii = real(poses(:,(1+ii*4))); % expressed in W
            R_CjCi_guess(:,:,ii+1) = R_W_C_ii';
            Cj_t_CjCi_guess(:,:,ii+1) = -R_W_C_ii' * t_W_C_ii;
        end
    else
        M_C_W_guess = estimatePoseDLT(keypoint_sample', landmark_sample', K);
        R_CjCi_guess = M_C_W_guess(:,1:3);
        Cj_t_CjCi_guess = M_C_W_guess(:,end);
    end
    
    % count inliers
    projected_points_uv = projectPoints((R_CjCi_guess(:,:,1)*Ci_corresponding_landmarks) +...
                                        repmat(-Cj_t_CjCi_guess(:,:,1),[1 size(Ci_corresponding_landmarks, 2)]), K);
    difference = matched_query_keypoints_uv - projected_points_uv;
    errors = sum(difference.^2,1);
    inliers = errors < params.localization_ransac.pixel_tolerance^2;
    
    if params.localization_ransac.use_p3p
        projected_points_uv = projectPoints((R_CjCi_guess(:,:,2) * Ci_corresponding_landmarks) +...
                                            repmat(-Cj_t_CjCi_guess(:,:,2),[1 size(Ci_corresponding_landmarks, 2)]), K);
        difference = matched_query_keypoints_uv - projected_points_uv;
        errors = sum(difference.^2, 1);
        alternative_inliers = errors < params.localization_ransac.pixel_tolerance^2;
        if nnz(alternative_inliers) > nnz(inliers)
            inliers = alternative_inliers;
            R_CjCi_guess(:,:,1) = R_CjCi_guess(:,:,2);
            Cj_t_CjCi_guess(:,:,1) = Cj_t_CjCi_guess(:,:,2);
        end
    end
    
    % save new model if better then old one
    if (nnz(inliers) > max_num_inliers && nnz(inliers) >= 6)
        max_num_inliers = nnz(inliers);        
        best_guess_inliers = inliers;
        R_CjCi_best_guess = R_CjCi_guess(:,:,1);
        Cj_t_CjCi_best_guess = Cj_t_CjCi_guess(:,:,1);
    end

    max_num_inliers_history(i) = max_num_inliers;
end

% display count of inliers evolution
if params.localization_ransac.show_iterations
    figure(fig_RANSAC_debug);
    plot(max_num_inliers_history);
    axis([0 num_iterations 0 size(matched_query_keypoints_uv,2)]);
    title('Max num inliers over iterations');
    
    % display fraction of inlier matches
    updateConsole(params,...
                  sprintf('  Max number of inlier matches found: %i (%0.2f perc.)\n',...
                  max_num_inliers,100*max_num_inliers/size(matched_query_keypoints_uv,2)));
end

% final tranformation calculation
Ci_corresponding_inlier_landmarks = [];

if (max_num_inliers == 0)
    matched_query_keypoints_uv = [];       
    R_CjCi = [];
    Cj_t_CjCi = [];
    updateConsole(params, '  No inlier matches found\n');
else
    % discard outliers
    matched_query_keypoints_uv = matched_query_keypoints_uv(:,best_guess_inliers);
    Ci_corresponding_inlier_landmarks = Ci_corresponding_landmarks(:,best_guess_inliers);

    % calculate [R,t] with best inlier points and DLT
    M_CjCi = estimatePoseDLT(matched_query_keypoints_uv', Ci_corresponding_inlier_landmarks', K);
    R_CjCi = M_CjCi(:,1:3);
    Cj_t_CjCi = M_CjCi(:,end);
	% R_CjCi = R_CjCi_best_guess;
    % Cj_t_CjCi = Cj_t_CjCi_best_guess;
end

% display projected keypoints given best pose and inlier correspondences
if (max_num_inliers > 0 && params.localization_ransac.show_projected_keypoints)
    
    Cj_best_guess_projected_pts = projectPoints((R_CjCi_best_guess*Ci_corresponding_inlier_landmarks) + ...
                                                repmat(-Cj_t_CjCi_best_guess,[1 size(Ci_corresponding_inlier_landmarks, 2)]), K);
                                            
    figure(fig_cont);
    subplot(3,1,1);
    plotPoints(flipud(Cj_best_guess_projected_pts),'yx');
    title('Projected keypoints in Cj-Frame (yellow crosses)');
end

%% Post processing
% calculate inverse rotation matrices
R_CiCj = R_CjCi';
Ci_t_CiCj = -R_CiCj*Cj_t_CjCi;

% flip keypoints back to restore [v u] order
matched_query_inlier_keypoints = flipud(matched_query_keypoints_uv);

% update gui inlier keypoints
if (params.through_gui && params.gui.show_inlier_features)
    gui_updateKeypoints(matched_query_keypoints, gui_handles.ax_current_frame, 'g.');
end

end
