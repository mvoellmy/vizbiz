function [query_keypoints, matches] = findCorrespondeces_cont(params, database_image, database_keypoints, query_image)
% Detects N keypoint correspondeces given image pair
% and returns the sorted keypoints of both images.
% 
% Input:
%  - params(struct) : parameter struct
%  - database_image(size) : first image
%  - query_image(size) : second image
%
% Output:
%  - query_keypoints (2XN) : all new keypoints
%  - matches (1xN)  : index vector. if non-zero links to matching database
%    keypoints

global fig_cont;

% compute harris scores for query image
query_harris = harris(query_image,params.corr.harris_patch_size,params.corr.harris_kappa);

% compute keypoints for query image
query_keypoints = selectKeypoints(query_harris,params.corr.num_keypoints,params.corr.nonmaximum_supression_radius);

% descripe query keypoints
query_descriptors = describeKeypoints(query_image,query_keypoints,params.corr.descriptor_radius);

% describe database keypoints
database_descriptors = describeKeypoints(database_image,database_keypoints,params.corr.descriptor_radius);

% match descriptors
matches = matchDescriptors(query_descriptors,database_descriptors,params.corr.match_lambda);

% % extract indices
% query_indeces = 1:length(matches);
% database_indeces = 1:size(database_keypoints,2);
% matched_query_indeces = query_indeces(matches > 0);
% %matched_database_indeces = database_indeces(matches > 0);
% 
% % filter invalid matches
% valid_matches = matches(matches > 0);
% 
% matched_query_keypoints = query_keypoints(:,matched_query_indeces);
% 
% % remove unseen dbkeypoints & sort database keypoints
% matched_database_keypoints = database_keypoints(:,valid_matches);
% 
% matched_database_indices = database_indeces(valid_matches);
% 
% % TODO
% % verify that the output vector 'valid matches' links to keypoints in 
% % 'matched_database_keypoints' that actually exist.  
% 
% valid_matches_cleaned = zeros(size(valid_matches));
% 
% for i = 1:size(valid_matches,2)
%     [ind,~] = find(matched_database_indices == valid_matches(i));
%     valid_matches_cleaned(i) = ind;
% end
% 
% 
% % display valid correspondences
% if params.cont.show_new_keypoints
%     figure(fig_cont);
%     imshow(query_image);
%     hold on;
%     plotPoints(query_keypoints,'r.');
%     if params.cont.show_matches
%         plotMatches(matches,query_keypoints,database_keypoints,'m-');
%         title('Current frame: Matches found');
%     end
% end

end
