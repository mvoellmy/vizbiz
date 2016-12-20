function [matched_database_keypoints, matched_query_keypoints, corr_ldk_matches] = ...
    findCorrespondeces_cont(params, database_image, database_keypoints, query_image)
% TODO description
% 
% Input:
%  - params(struct) : parameter struct
%  - database_image(size) : first image
%  - database_keypoints(2xN) : previous image keypoints, [v u]
%  - query_image(size) : second image
%
% Output:
%  - matched_database_keypoints(2xN) : matched keypoints of first image, [v u]
%  - matched_query_keypoints(2xN) : matched keypoints of second image, [v u]
%  - corr_ldk_matches(1xN) : indices of landmarks corresponding to matched keypoints

global fig_cont gui_handles;

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

% display fraction of matched keypoints
fprintf('  Number of new keypoints matched with prev keypoints: %i (%0.2f %%)\n',...
        nnz(matches),100*nnz(matches)/size(database_keypoints,2));

% filter invalid matches
[~,matched_query_indices,matched_database_indices] = find(matches);
matched_query_keypoints = query_keypoints(:,matched_query_indices);
matched_database_keypoints = database_keypoints(:,matched_database_indices);
corr_ldk_matches = matches(matches > 0); % only for link to landmark !!!

% check for consistent correspondences
assert(size(matched_query_keypoints,2) == length(corr_ldk_matches) && ...
       size(matched_database_keypoints,2) == length(corr_ldk_matches));

% display valid correspondences
if params.cont.show_new_keypoints
    figure(fig_cont);
    subplot(2,1,1);
    imshow(query_image);
    hold on;
    plotPoints(query_keypoints,'r.');
    if params.cont.show_matches
        plotMatches(matches,query_keypoints,database_keypoints,'m-');
        title('Matches found');
    end
    subplot(2,1,2);
    imshow(query_image);
    hold on;
end

% update gui image
gui_updateImage(query_image, gui_handles.ax_current_frame);

% update gui keypoints
if params.gui.show_all_features
    gui_updateKeypoints(query_keypoints, gui_handles.ax_current_frame, 'r.');
end

end
