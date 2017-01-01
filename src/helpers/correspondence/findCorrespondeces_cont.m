function [query_keypoints, matches, matched_query_keypoints, matched_database_keypoints] = ...
    findCorrespondeces_cont(params, database_image, database_keypoints, query_image)
% TODO description
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
%  - matches(2xN):  indices vector where the i-th coefficient is the index of
%    database_keypoints which matches to the i-th entry of matched_query_keypoints.
%  - matched_query_keypoints : todo
%  - matched_database_keypoints : todo

global gui_handles;

% compute harris scores for query image
query_harris = harris(query_image,params.corr.harris_patch_size,params.corr.harris_kappa);

% compute keypoints for query image
query_keypoints = selectKeypoints(query_harris,params.corr.num_keypoints,params.corr.nonmaximum_supression_radius);

% descripe query keypoints
query_descriptors = describeKeypoints(query_image,query_keypoints,params.corr.descriptor_radius);

% describe database keypoints
database_descriptors = describeKeypoints(database_image,database_keypoints,params.corr.descriptor_radius);

if params.cont.use_KLT
    % Create a point tracker and enable the bidirectional error constraint to
    % make it more robust in the presence of noise and clutter.
    kp_tracker = vision.PointTracker('NumPyramidLevels', 4, 'MaxBidirectionalError', 2);
    
    % initialize tracker with the query kp locations
    initialize(kp_tracker, flipud(database_keypoints)', database_image);
    
    % track keypoints
    [tracked_kp, matches, ~] = step(kp_tracker, query_image); % todo: use validity scores?
    matched_query_keypoints = flipud(tracked_kp(matches,:)');
    matched_database_keypoints = database_keypoints(:,matches);
else
    % match descriptors
    matches = matchDescriptors(query_descriptors,database_descriptors,params.corr.match_lambda);
    
    % filter query keypoints and database keypoints
    [~, matched_query_indices, matched_database_indices] = find(matches);
    matched_query_keypoints = query_keypoints(:,matched_query_indices);
    matched_database_keypoints = database_keypoints(:,matched_database_indices);
end

% display fraction of matched keypoints
updateConsole(params,...
              sprintf('  Number of new keypoints matched with prev keypoints: %i (%0.2f perc.)\n',...
              nnz(matches), 100*nnz(matches)/size(database_keypoints,2)));

% update gui image
if params.through_gui
    gui_updateImage(query_image, gui_handles.ax_current_frame);
    
    % update gui keypoints
    if params.gui.show_all_features
        gui_updateKeypoints(query_keypoints, gui_handles.ax_current_frame, 'r.');
    end
end

end
