function [matched_database_keypoints, matched_query_keypoints, query_keypoints] = ...
    findCorrespondeces(params, database_image, query_image)
% Detects N keypoint correspondeces given image pair
% and returns the SORTED keypoints of both images.
% 
% Input:
%  - params(struct) : parameter struct
%  - database_image(size) : first image
%  - query_image(size) : second image
%
% Output:
%  - matched_database_keypoints(2xN) : matched keypoints of first image,  each [u v]
%  - matched_query_keypoints(2xN) : matched keypoints of second image, each  [u v]
%  - query_keypoints(2xN): All query keypoints found [v u] !!!!

global fig_init gui_handles;

% todo: move params into subroutines, expose only method string
% compute harris scores for query image % todo: move into keypoint-selection
query_harris = harris(query_image,params.init.corr.harris_patch_size,params.init.corr.harris_kappa);

% compute harris scores for query image
database_harris = harris(database_image,params.init.corr.harris_patch_size,params.init.corr.harris_kappa);

% compute keypoints for query image
query_keypoints = selectKeypoints(query_harris,params.init.corr.num_keypoints,params.init.corr.nonmaximum_supression_radius);

% compute keypoints for database image
database_keypoints = selectKeypoints(database_harris,params.init.corr.num_keypoints,params.init.corr.nonmaximum_supression_radius);

% descripe query keypoints
query_descriptors = describeKeypoints(query_image,query_keypoints,params.init.corr.descriptor_radius);

% describe database keypoints
database_descriptors = describeKeypoints(database_image,database_keypoints,params.init.corr.descriptor_radius);

% match descriptors
matches = matchDescriptors(query_descriptors,database_descriptors,params.init.corr.match_lambda);

% display fraction of matched keypoints
updateConsole(params,...
              sprintf('  Number of new keypoints matched with prev keypoints: %i (%0.2f perc.)\n',...
              nnz(matches), 100*nnz(matches)/size(database_keypoints,2)));

% filter invalid matches
[~, matched_query_indices, matched_database_indices] = find(matches);
matched_query_keypoints = flipud(query_keypoints(:,matched_query_indices));
matched_database_keypoints = flipud(database_keypoints(:,matched_database_indices));

% check for consistent correspondences
assert(size(matched_query_keypoints,2) == size(matched_database_keypoints,2));

% display bootstrap pair keypoints % todo: move to initPipeline()
if (params.init.figures && params.init.show_keypoints)
    fig_init = figure('name', 'Initialization');
    
    subplot(2,2,1);
    imshow(database_image);
    hold on;
    plotPoints(database_keypoints,'rx');
    title('Initialization frame 1 keypoints');
    
    subplot(2,2,2);
    imshow(query_image);
    hold on;
    plotPoints(query_keypoints,'rx');
    title('Initialization frame 2 keypoints');
    
    % display valid correspondences
    if params.init.show_matches
        subplot(2,2,3);
        showMatchedFeatures(database_image, query_image,...
                            matched_database_keypoints',...
                            matched_query_keypoints', 'blend', 'PlotOptions', {'rx','rx','m-'});
        title('Candidate keypoint matches');     
    end
end

% update gui keypoints % todo: move to initPipeline()
if params.through_gui && params.gui.show_all_features
    gui_updateKeypoints(query_keypoints, gui_handles.ax_current_frame, 'r.');
end

end
