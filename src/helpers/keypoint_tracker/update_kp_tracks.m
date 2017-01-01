function [ kp_tracks_updated ] = update_kp_tracks(params, kp_tracks_prev,img_prev, img_new, query_keypoints, T_WCj, fig_kp_tracks)
% Tries to match query keypoints with current image and keeps track of
% tracked candiate keypoints and their first observations.

% Inserts new candidate keypoints into the keypoint tracks and discard
% candidate keypoints that could not be matched.

% variable init - assume no matches
matches_untriang = zeros(1,size(query_keypoints,2));
kp_tracks_updated.candidate_kp = [];   % 2xN
kp_tracks_updated.first_obs_kp = [];   % 2xN
kp_tracks_updated.first_obs_pose = []; % 16xN
kp_tracks_updated.nr_trackings = []; % 1xN

% if there are candiate keypoints, try to match
if (size(kp_tracks_prev.candidate_kp,2) > 0) % 0 in first frame
    % descripe query keypoints
    query_descriptors = describeKeypoints(img_new,query_keypoints,params.corr.descriptor_radius);

    % describe database keypoints
    database_descriptors = describeKeypoints(img_prev,kp_tracks_prev.candidate_kp,params.corr.descriptor_radius);
    
    % match descriptors
    matches_untriang = matchDescriptors(query_descriptors,database_descriptors,params.corr.match_lambda);
    % OPTIONAL TODO: Lucas kanade?
    
    new_kp_1 = query_keypoints(:,matches_untriang==0);
    new_kp_2 = [];
    
    % Remove match outliers
%     fprintf('------------>Number of keypoints before RANSAC: %d\n', nnz(matches_untriang));
%      [query_keypoints, kp_tracks_prev.candidate_kp, matches_untriang, new_kp_2] = ...
%          eightPointRansac_cont(params, query_keypoints, kp_tracks_prev.candidate_kp, matches_untriang, K, K);
%     fprintf('------------>Number of keypoints after  RANSAC: %d\n', nnz(matches_untriang));
%  
    % update candidate_kp coordinates with matched current kp
    idx_matched_kp_tracks_cand = find(matches_untriang); % z.B 17
    idx_matched_kp_tracks_database = matches_untriang(matches_untriang>0);

    % update kp information that could be tracked (sorting like query keypoints)
    kp_tracks_updated.candidate_kp(:,idx_matched_kp_tracks_cand) = query_keypoints(:,idx_matched_kp_tracks_cand); %new v,u coord
    kp_tracks_updated.first_obs_kp(:,idx_matched_kp_tracks_cand) = kp_tracks_prev.first_obs_kp(:,idx_matched_kp_tracks_database);
    kp_tracks_updated.first_obs_pose(:,idx_matched_kp_tracks_cand) = kp_tracks_prev.first_obs_pose(:,idx_matched_kp_tracks_database);
    kp_tracks_updated.nr_trackings(idx_matched_kp_tracks_cand) = kp_tracks_prev.nr_trackings(idx_matched_kp_tracks_database)+1;

    % discard kp that could not be tracked --> new sorting (unkown)
    kp_tracks_updated.candidate_kp = kp_tracks_updated.candidate_kp(:,idx_matched_kp_tracks_cand);
    kp_tracks_updated.first_obs_kp = kp_tracks_updated.first_obs_kp(:,idx_matched_kp_tracks_cand);
    kp_tracks_updated.first_obs_pose = kp_tracks_updated.first_obs_pose(:,idx_matched_kp_tracks_cand);
    kp_tracks_updated.nr_trackings = kp_tracks_updated.nr_trackings(idx_matched_kp_tracks_cand);

    % append all new found keypoints and their pose
    new_kp = [new_kp_1, new_kp_2]; % kp which could not be matched
else
    new_kp = query_keypoints(:,matches_untriang==0);
end

T_WCj_col = T_WCj(:); % convert to col vector for storage
kp_tracks_updated.candidate_kp = [kp_tracks_updated.candidate_kp, new_kp];
kp_tracks_updated.first_obs_kp = [kp_tracks_updated.first_obs_kp, new_kp]; % is equal to candidate when adding
kp_tracks_updated.first_obs_pose = [kp_tracks_updated.first_obs_pose, repmat(T_WCj_col,[1, size(new_kp, 2)])];
kp_tracks_updated.nr_trackings = [kp_tracks_updated.nr_trackings, zeros(1, size(new_kp, 2))];

% display matched keypoint tracks
if params.keypoint_tracker.show_matches
    figure(fig_kp_tracks);
    if (size(kp_tracks_prev.candidate_kp,2) > 0) % 0 in first frame
        plotPoints(kp_tracks_prev.candidate_kp,'r.');
        plotMatches(matches_untriang,query_keypoints,kp_tracks_prev.candidate_kp,'m-');
    end
    plotPoints(kp_tracks_updated.candidate_kp,'y.');

    %title('Candidate Keypoints: Old (red), updated (yellow), Matches');
end

fprintf('  Number of matched keypoint candidates: %i (%0.2f %%)\n'...
         ,nnz(matches_untriang),100*nnz(matches_untriang)/size(kp_tracks_prev.candidate_kp,2)); 

end

