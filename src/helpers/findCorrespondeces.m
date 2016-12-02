function [database_keypoints, query_keypoints, corresponding_matches] = findCorrespondeces(p, database_image, query_image)
% Detects k keypoints correspondeces
% Returns the sorted keypoints of both images
% 
% Input: point correspondences
%  - 
%
% Output:
%  - database_keypoints(2,N) : keypoints of first image
%  - query_keypoints(2,N) : keypoints of second image


% compute harris scores for query image
query_harris = harris(query_image, p.corr.harris_patch_size, p.corr.harris_kappa);

% compute harris scores for query image
database_harris = harris(database_image, p.corr.harris_patch_size, p.corr.harris_kappa);

% compute keypoints for query image
query_keypoints = selectKeypoints(query_harris, p.corr.num_keypoints, p.corr.nonmaximum_supression_radius);

% compute keypoints for database image
database_keypoints = selectKeypoints(database_harris, p.corr.num_keypoints, p.corr.nonmaximum_supression_radius);

% descripe query keypoints
query_descriptors = describeKeypoints(query_image, query_keypoints, p.corr.descriptor_radius);

% describe database keypoints
database_descriptors = describeKeypoints(database_image, database_keypoints, p.corr.descriptor_radius);

% match descriptors
all_matches = matchDescriptors(query_descriptors, database_descriptors, p.corr.match_lambda); % 

% filter invalid matches
corresponding_matches = all_matches(all_matches > 0); % maybe unnecessary

% OPTIONAL filter outlier matches with RANSAC



end