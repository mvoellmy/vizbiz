function [matched_database_keypoints, matched_query_keypoints] = ...
    findCorrespondeces_boot(params, database_image, database_keypoints, query_image)
% Returns matched keypoints given database keypoints, already sorted and
% filtered.
% 
% Input:
%  - params(struct) : parameter struct
%  - database_image(size) : first image
%  - database_keypoints(2xN) : previous image keypoints, [v u] which have a
%    corresponding landmark
%  - query_image(size) : second image
%
% Output:
%  - query_keypoints(2xN) : matched keypoints of second image, [v u]
%  - matches (2xN):  indices vector where the i-th coefficient is the index of
%    database_keypoints which matches to the i-th entry of matched_query_keypoints.

global fig_boot gui_handles;

% compute harris scores for query image
query_harris = harris(query_image,params.corr.harris_patch_size,params.corr.harris_kappa);

% compute keypoints for query image
query_keypoints = selectKeypoints(query_harris,params.boot.num_keypoints,params.corr.nonmaximum_supression_radius);

% descripe query keypoints
query_descriptors = describeKeypoints(query_image,query_keypoints,params.corr.descriptor_radius);

% describe database keypoints
database_descriptors = describeKeypoints(database_image,database_keypoints,params.corr.descriptor_radius);

% match descriptors
matches = matchDescriptors(query_descriptors,database_descriptors,params.corr.match_lambda);

% display fraction of matched keypoints
updateConsole(params,...
              sprintf('  Number of new keypoints matched with prev keypoints: %i (%0.2f perc.)\n',...
              nnz(matches),100*nnz(matches)/size(database_keypoints,2)));

% filter invalid matches
[~,matched_query_indices,matched_database_indices] = find(matches);
matched_query_keypoints = flipud(query_keypoints(:,matched_query_indices));
matched_database_keypoints = flipud(database_keypoints(:,matched_database_indices));

% check for consistent correspondences
assert(size(matched_query_keypoints,2) == size(matched_database_keypoints,2));

% display bootstrap pair keypoints
if params.boot.show_boot_keypoints
    figure(fig_boot);
    
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
    if params.boot.show_matches
        subplot(2,2,3);
        showMatchedFeatures(database_image,query_image,matched_database_keypoints',matched_query_keypoints');
        title('Bootstrap frame pair matches');
        
        subplot(2,2,4);
        imshow(query_image);
        hold on;
        plotPoints(flipud(matched_query_keypoints),'r.');
        plotMatches(matches,query_keypoints,database_keypoints,'m-');
        title('Initialization image with matches');
    end
end

% update gui keypoints
if params.gui.show_all_features
    gui_updateKeypoints(query_keypoints, gui_handles.ax_current_frame, 'r.');
end

end
