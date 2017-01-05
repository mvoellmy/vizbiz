function [ kp_tracks_updated ] = updateKpTracks(params, kp_tracks_prev,img_prev, img_new, query_keypoints, T_WCj)
% Tries to match query keypoints with current image and keeps track of
% tracked candiate keypoints and their first observations.
% Inserts new candidate keypoints into the keypoint tracks and discard
% candidate keypoints that could not be matched.

% Inputs:
% - query_keypoints (2xN) : [v u]
% - T_WCj(4x4) : Trasnformation matrix 

global fig_kp_tracks;

% variable init - assume no matches
matches_untriang = zeros(1,size(query_keypoints,2));
kp_tracks_updated.candidate_kp = [];   % 2xN
kp_tracks_updated.first_obs_kp = [];   % 2xN
kp_tracks_updated.first_obs_pose = []; % 16xN
kp_tracks_updated.nr_trackings = []; % 1xN

% if there are candiate keypoints, try to match
if (size(kp_tracks_prev.candidate_kp,2) > 0) % 0 in first frame
    if params.kp_tracker.use_KLT
        % create a point tracker
        klt_tracker = vision.PointTracker('NumPyramidLevels', 4, 'MaxBidirectionalError', 2);

        % initialize tracker with the query kp locations
        initialize(klt_tracker, flipud(kp_tracks_prev.candidate_kp)', img_prev);

        % set points
        %setPoints(klt_tracker, flipud(query_keypoints)');
        
        % track keypoints
        [kp_tracked, validIdx, ~] = step(klt_tracker, img_new); % todo: use validity scores?
        matches_untriang = validIdx'.*(1:size(kp_tracks_prev.candidate_kp,2));
        
        % update candidate_kp coordinates with matched current kp
        idx_matched_kp_tracks_cand = find(matches_untriang);
        idx_matched_kp_tracks_database = idx_matched_kp_tracks_cand;
        
        % update kp information that could be tracked (sorting like query keypoints)
        kp_tracks_updated.candidate_kp(:,idx_matched_kp_tracks_cand) = flipud(kp_tracked(validIdx,:)'); % new v,u coord
        kp_tracks_updated.first_obs_kp(:,idx_matched_kp_tracks_cand) = kp_tracks_prev.first_obs_kp(:,idx_matched_kp_tracks_database);
        kp_tracks_updated.first_obs_pose(:,idx_matched_kp_tracks_cand) = kp_tracks_prev.first_obs_pose(:,idx_matched_kp_tracks_database);
        kp_tracks_updated.nr_trackings(idx_matched_kp_tracks_cand) = kp_tracks_prev.nr_trackings(idx_matched_kp_tracks_database)+1;
        
        % append all new found keypoints and their pose
        %new_kp = query_keypoints(:,matches_untriang==0);
        
        % append all new found keypoints and their pose
        % BUT only if they lie within image boundaries
        % todo: maybe add extra features with describeKeypoints
        kp_tracked_untriang = kp_tracked(~validIdx,:);
        [within_boundsIdx, ~] = find((kp_tracked_untriang(:,1) > 0) & (kp_tracked_untriang(:,1) < size(img_new,2)) &...
                                     (kp_tracked_untriang(:,2) > 0) & (kp_tracked_untriang(:,2) < size(img_new,1)) );
        kp_tracked_untriang_within_bounds = kp_tracked_untriang(within_boundsIdx,:);
        
        new_kp = flipud(kp_tracked_untriang_within_bounds');
        
        assert(nnz(new_kp < 0) == 0, 'invalid new kp');
    else
        % descripe query keypoints
        query_descriptors = describeKeypoints(img_new,query_keypoints,params.corr.descriptor_radius);

        % describe database keypoints
        database_descriptors = describeKeypoints(img_prev,kp_tracks_prev.candidate_kp,params.corr.descriptor_radius);

        % match descriptors
        matches_untriang = matchDescriptors(query_descriptors,database_descriptors,params.corr.match_lambda);

        % update candidate_kp coordinates with matched current kp
        idx_matched_kp_tracks_cand = find(matches_untriang);
        idx_matched_kp_tracks_database = matches_untriang(matches_untriang>0);
        
        % update kp information that could be tracked (sorting like query keypoints)
        kp_tracks_updated.candidate_kp(:,idx_matched_kp_tracks_cand) = query_keypoints(:,idx_matched_kp_tracks_cand); % new v,u coord
        kp_tracks_updated.first_obs_kp(:,idx_matched_kp_tracks_cand) = kp_tracks_prev.first_obs_kp(:,idx_matched_kp_tracks_database);
        kp_tracks_updated.first_obs_pose(:,idx_matched_kp_tracks_cand) = kp_tracks_prev.first_obs_pose(:,idx_matched_kp_tracks_database);
        kp_tracks_updated.nr_trackings(idx_matched_kp_tracks_cand) = kp_tracks_prev.nr_trackings(idx_matched_kp_tracks_database)+1;
        
        % append all new found keypoints and their pose
        new_kp = query_keypoints(:,matches_untriang==0);
    end
    
    % discard kp that could not be tracked --> new sorting (unknown)
    kp_tracks_updated.candidate_kp = kp_tracks_updated.candidate_kp(:,idx_matched_kp_tracks_cand);
    kp_tracks_updated.first_obs_kp = kp_tracks_updated.first_obs_kp(:,idx_matched_kp_tracks_cand);
    kp_tracks_updated.first_obs_pose = kp_tracks_updated.first_obs_pose(:,idx_matched_kp_tracks_cand);
    kp_tracks_updated.nr_trackings = kp_tracks_updated.nr_trackings(idx_matched_kp_tracks_cand);
else
    new_kp = query_keypoints(:,matches_untriang==0);
end

T_WCj_col = T_WCj(:); % convert to col vector for storage

kp_tracks_updated.candidate_kp = [kp_tracks_updated.candidate_kp, new_kp];
kp_tracks_updated.first_obs_kp = [kp_tracks_updated.first_obs_kp, new_kp]; % is equal to candidate when adding
kp_tracks_updated.first_obs_pose = [kp_tracks_updated.first_obs_pose, repmat(T_WCj_col,[1, size(new_kp, 2)])];
kp_tracks_updated.nr_trackings = [kp_tracks_updated.nr_trackings, zeros(1, size(new_kp, 2))];

updateConsole(params,...
              sprintf('  Number of matched keypoint candidates: %i (%0.2f perc.)\n',...
              nnz(matches_untriang),100*nnz(matches_untriang)/size(kp_tracks_prev.candidate_kp,2))); 

% display matched keypoint tracks
if (params.cont.figures && params.kp_tracker.show_matches)
    figure(fig_kp_tracks);
    if (size(kp_tracks_prev.candidate_kp,2) > 0) % 0 in first frame
        plotPoints(kp_tracks_prev.candidate_kp,'r.');
        %plotMatches(matches_untriang,query_keypoints,kp_tracks_prev.candidate_kp,'m-');
    end
    plotPoints(kp_tracks_updated.candidate_kp,'y.');
    title('Previous kp (red), Updated kp (yellow)');
end

end
