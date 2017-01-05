function [query_keypoints,matched_query_indices, matched_keypoints, matchedLandmarks] = ...
    findCorrespondeces_cont(params, database_image, database_keypoints, query_image, database_landmarks)
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
%  - matched_keypoints(2xM) : filtered, tracked database keypoints [v u]
%  - matchedLandmarks(3xM) : filtered, tracked previous landmarks

global fig_cont gui_handles;

% compute harris scores for query image
query_harris = harris(query_image,params.corr.harris_patch_size,params.corr.harris_kappa);

% compute keypoints for query image
query_keypoints = selectKeypoints(query_harris,params.corr.num_keypoints,params.corr.nonmaximum_supression_radius);
matched_query_indices = zeros(1,size(query_keypoints,2));

if params.cont.use_KLT
    % create a point tracker
    klt_tracker = vision.PointTracker();

    % initialize tracker with the query kp locations
    initialize(klt_tracker, flipud(database_keypoints)', database_image);

    % track keypoints
    [kp_tracked, validIdx, ~] = step(klt_tracker, query_image); % todo: use validity scores?
    query_keypoints_klt = round(flipud(kp_tracked')); % [v u]
     
    % matched_database_keypoints = flipud(database_keypoints(:,matches));
    matched_keypoints = query_keypoints_klt(:,validIdx'); % [v u]
    
    % Filter coresponding landmarks
    matchedLandmarks = database_landmarks(:,validIdx');
       
else
    % descripe query keypoints
    query_descriptors = describeKeypoints(query_image,query_keypoints,params.corr.descriptor_radius);

    % describe database keypoints
    database_descriptors = describeKeypoints(database_image,database_keypoints,params.corr.descriptor_radius);

    % match descriptors
    matches = matchDescriptors(query_descriptors,database_descriptors,params.corr.match_lambda);
    
    % filter query and database keypoints
    [~, matched_query_indices, matched_database_indices] = find(matches);
    matched_keypoints = query_keypoints(:,matched_query_indices);
    
    % delete landmark where no matching keypoint was found
    corr_ldk_matches = matches(matches > 0);
    matchedLandmarks = database_landmarks(:,corr_ldk_matches);
end

% display all correspondences
if (params.cont.figures && params.cont.show_new_keypoints)
    figure(fig_cont);
    subplot(2,1,1);
    imshow(query_image);
    hold on;
    plotPoints(query_keypoints,'r.');
    if params.cont.show_matches
        %plotMatches(matches,query_keypoints,database_keypoints,'m-');
        title('Matches found');
    end
    subplot(2,1,2);
    imshow(query_image);
    hold on;
end

% update gui image
if params.through_gui
    gui_updateImage(query_image, gui_handles.ax_current_frame);
end

% update gui keypoints
if params.through_gui && params.gui.show_all_features
    gui_updateKeypoints(query_keypoints, gui_handles.ax_current_frame, 'r.');
end

end
