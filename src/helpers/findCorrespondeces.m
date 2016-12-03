function [matched_database_keypoints, matched_query_keypoints, valid_matches] = findCorrespondeces(params, database_image, query_image)
% Detects N keypoint correspondeces given image pair
% and returns the sorted keypoints of both images
% 
% Input:
%  - params(struct) : parameter struct
%  - database_image(size) : first image
%  - query_image(size) : second image
%
% Output:
%  - database_keypoints(2xN) : matched keypoints of first image
%  - query_keypoints(2xN) : matched keypoints of second image
%  - matches(1xN) : indeces of query keypoints matched with db keypoints

% compute harris scores for query image
query_harris = harris(query_image,params.corr.harris_patch_size,params.corr.harris_kappa);

% compute harris scores for query image
database_harris = harris(database_image,params.corr.harris_patch_size,params.corr.harris_kappa);

% compute keypoints for query image
query_keypoints = selectKeypoints(query_harris,params.corr.num_keypoints,params.corr.nonmaximum_supression_radius);

% compute keypoints for database image
database_keypoints = selectKeypoints(database_harris,params.corr.num_keypoints,params.corr.nonmaximum_supression_radius);

% descripe query keypoints
query_descriptors = describeKeypoints(query_image,query_keypoints,params.corr.descriptor_radius);

% describe database keypoints
database_descriptors = describeKeypoints(database_image,database_keypoints,params.corr.descriptor_radius);

% match descriptors
matches = matchDescriptors(query_descriptors,database_descriptors,params.corr.match_lambda);

% extract indices
query_indeces = 1:length(matches);
database_indeces = matches;
matched_query_indeces = query_indeces(matches > 0);
matched_database_indeces = database_indeces(matches > 0);

% filter invalid matches
valid_matches = matches(matches > 0);
matched_query_keypoints = query_keypoints(:,matched_query_indeces);
matched_database_keypoints = database_keypoints(:,matched_database_indeces);

% todo: filter outlier matches with RANSAC

if params.corr.show_corr_matches
    figure('name','Matches found in second frame');
    imshow(query_image);
    hold on;
    plotPoints(query_keypoints);
    if ~params.init.use_KITTI_precalculated_init
        plotMatches(matches, query_keypoints, database_keypoints);
    end
end

end
