function [R_C_W, t_C_W, matched_query_keypoints, matched_database_keypoints, corresponding_matches, max_num_inliers_history] = ransacLocalization(params, ...
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

global fig_cont;

% find 2D correspondences
[matched_database_keypoints,matched_query_keypoints,corresponding_matches] = findCorrespondeces_cont(params,database_image,database_keypoints,query_image);
corresponding_landmarks = p_W_landmarks(:,corresponding_matches);

% display matched keypoints
if params.localization_ransac.show_matched_keypoints
    figure(fig_cont);
    plotPoints(matched_query_keypoints,'g.');
    title('Current frame: Revisited keypoints');
end

% homogenize points
p_hom_i1 = [matched_database_keypoints; ones(1,length(matched_database_keypoints))];
p_hom_i2 = [matched_query_keypoints; ones(1,length(matched_query_keypoints))];

% choose RANSAC options
if params.localization_ransac.use_p3p
    num_iterations = 200;
    pixel_tolerance = 10;
    s = 3;
else
    num_iterations = 2000;
    pixel_tolerance = 10;
    s = 6;
end

% initialize RANSAC
inliers = zeros(1, size(matched_query_keypoints,2));
matched_query_keypoints = flipud(matched_query_keypoints); % ??
max_num_inliers_history = zeros(1,num_iterations);
max_num_inliers = 0;

% run RANSAC for pose estimation
for i = 1:num_iterations
    [landmark_sample, idx] = datasample(...
        corresponding_landmarks, s, 2, 'Replace', false);
    keypoint_sample = matched_query_keypoints(:, idx);
    
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
    is_inlier = errors < pixel_tolerance^2;
    
    if params.localization_ransac.use_p3p
        projected_points = projectPoints(...
            (R_C_W_guess(:,:,2) * corresponding_landmarks) + ...
            repmat(t_C_W_guess(:,:,2), ...
            [1 size(corresponding_landmarks, 2)]), K);
        difference = matched_query_keypoints - projected_points;
        errors = sum(difference.^2, 1);
        alternative_is_inlier = errors < pixel_tolerance^2;
        if nnz(alternative_is_inlier) > nnz(is_inlier)
            is_inlier = alternative_is_inlier;
        end
    end
    
    % save new model if better then old one
    if nnz(is_inlier) > max_num_inliers && nnz(is_inlier) >= 6
        max_num_inliers = nnz(is_inlier);        
        inliers = is_inlier;
    end
    
    max_num_inliers_history(i) = max_num_inliers;
end

if (max_num_inliers == 0)
    R_C_W = [];
    t_C_W = [];
    fprintf('no inlier matches found\n');
else % calculate [R,T] with best inlier points
    M_C_W = estimatePoseDLT(...
        matched_query_keypoints(:, inliers>0)', ...
        corresponding_landmarks(:, inliers>0)', K);
    R_C_W = M_C_W(:,1:3);
    t_C_W = M_C_W(:,end);
end

% display inlier matches
if (nnz(inliers) > 0 && params.localization_ransac.show_inlier_matches)
    figure(fig_cont);
    plotMatches(1:nnz(inliers),p_hom_i2(1:2,inliers),p_hom_i1(1:2,inliers),'y-');
    title('Current frame: Inlier matches found');
end

end
