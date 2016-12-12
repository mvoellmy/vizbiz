function [matched_database_keypoints, matched_query_keypoints, valid_matches] = findCorrespondeces(params, database_image, query_image)
% Detects N keypoint correspondeces given image pair
% and returns the SORTED keypoints of both images.
% 
% Input:
%  - params(struct) : parameter struct
%  - database_image(size) : first image
%  - query_image(size) : second image
%
% Output:
%  - matched_database_keypoints(2xN) : matched keypoints of first image, each [u v]
%  - matched_query_keypoints(2xN) : matched keypoints of second image, each [u v]
%  - valid_matches(1xN) : !!! indeces of query keypoints matched with db keypoints

global fig_init;

% todo: move params into subroutines, expose only method string
% compute harris scores for query image % todo: move int keypoint-selection
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

% display fraction of matched keypoints
%fprintf('  %0.2f %% of keypoints matched\n',100*nnz(matches)/size(matches,2));
fprintf('  Number of new keypoints matched with prev keypoints: %i (%0.2f %%)\n',...
        nnz(matches),100*nnz(matches)/size(database_keypoints,2));

% filter invalid matches
[~,matched_query_indices,matched_database_indices] = find(matches);
matched_query_keypoints = flipud(query_keypoints(:,matched_query_indices));
matched_database_keypoints = flipud(database_keypoints(:,matched_database_indices));
valid_matches = 1:length(matched_query_indices); % still needed?

% check for consistent correspondences
% assert(size(matched_query_keypoints,2) == length(valid_matches) && ...
%        size(matched_database_keypoints,2) == length(valid_matches));
assert(size(matched_query_keypoints,2) == size(matched_database_keypoints,2));

% display bootstrap pair keypoints
if params.init.show_init_keypoints
    fig_init = figure('name','Keypoints found in bootstrap frame pair');
    
    subplot(2,2,1);
    imshow(database_image);
    hold on;
    plotPoints(database_keypoints,'rx');
    title('Bootstrap frame 1 keypoints');
    
    subplot(2,2,2);
    imshow(query_image);
    hold on;
    plotPoints(query_keypoints,'rx');
    title('Bootstrap frame 2 keypoints');
    
    % display valid correspondences
    if params.init.show_corr_matches        
        subplot(2,2,3);
        showMatchedFeatures(database_image,query_image,matched_database_keypoints',matched_query_keypoints');
        title('Bootstrap frame pair matches');
        
        subplot(2,2,4);
        imshow(query_image);
        hold on;
        plotPoints(flipud(matched_query_keypoints),'r.');
        plotMatches(matches,query_keypoints,database_keypoints,'m-');
        title('Initialitation image with matches');
    end
end

end
