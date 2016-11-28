function [database_keypoints, query_keypoints] = find_correspondeces(p, database_image, query_image)
% Detects k keypoints correspondeces
% Returns the sorted keypoints of both images
% 
% Input: point correspondences
%  - 
%
% Output:
%  - database_keypoints(k,3) : homogeneous keypoints of first image
%  - query_keypoints(k,3) : homogeneous keypoints of second image


% compute harris scores for query image
query_harris = harris(query_image, harris_patch_size, p.corr.harris_kappa);

% compute keypoints for query image
query_keypoints = selectKeypoints(query_harris, num_keypoints, p.corr.nonmaximum_supression_radius);

% compute keypoints for database image
database_keypoints = selectKeypoints(query_harris, num_keypoints, p.corr.nonmaximum_supression_radius);

% descripe query keypoints
query_descriptors = describeKeypoints(query_image, query_keypoints, p.corr.descriptor_radius);

% describe database keypoints
database_descriptors = describeKeypoints(database_image, database_keypoints, p.corr.descriptor_radius);

% match descriptors
all_matches = matchDescriptors(query_descriptors, database_descriptors, p.corr.match_lambda);

% filter invalid matches
corresponding_matches = all_matches(all_matches > 0);

% filter outlier matches with RANSAC

% TODO sort keypoints correctly

% homogenize keypoints

end