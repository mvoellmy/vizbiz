function [ Cj_P_hom_new_inliers, p_candidates_j_inliers, p_candidates_first_inliers,p_candidates_j_inliers_nr_tracking, kp_tracks_updated ] =...
    triangulateNewLandmarks(params, kp_tracks_updated, K , fig_kp_triangulate, fig_kp_tracks, T_WCj, nr_landmarks)
% Checks which candiate keypoints have good trianguability (bearing vector)
% and then triangulates a new landmark.
% Newly triangulated landmarks are filtered to have positive z-coordinate
% and their reprojetion error must be smaller then a defined threshold.
% The used candidate keypoints are removed from the keypoint tracks and the
% cleaned up keypoints tracks is returned.
%
% Input:
% - K: Camera calibration matrix
% - T_WCj (4x4): Transformation matrix
% - fig_kp_triangulate: Figure handle
% - nr_landmarks: Number of existing landmarks in pipeline
%
% Output:
% - Cj_P_hom_new_inliers [4xN]: New landmarks filtered homogenized
% - p_candidates_j_inliers [2xN]: Keypoints corresponding to landmarks [v u]
% todo: describe, use global figures

%% Triangulate new landmarks
% calculate bearing angle
bearing_angle_deg = calcBearingAngle(kp_tracks_updated.first_obs_kp, kp_tracks_updated.candidate_kp, K);
% vector_first = [(kp_tracks_updated.first_obs_kp); repmat(K(1,1),[1, size(kp_tracks_updated.first_obs_kp,2)])];
% vector_j = [kp_tracks_updated.candidate_kp; repmat(K(1,1),[1, size(kp_tracks_updated.candidate_kp,2)])];
% 
% bearing_angle_deg = atan2d(twoNormMatrix(cross(vector_j,vector_first)), dot(vector_j,vector_first));

% adapt bearing angle threshhold to landmarks remaining for triangulation
if nr_landmarks < params.kp_tracker.min_nr_landmarks_bearing_angle_adapt
    bearing_angle_low_thr = params.kp_tracker.bearing_low_thr;
    bearing_angle_up_thr = params.kp_tracker.bearing_up_thr;
else
    bearing_angle_low_thr = params.kp_tracker.bearing_low_thr * params.kp_tracker.bearing_angle_multiplicator;
    bearing_angle_up_thr = params.kp_tracker.bearing_up_thr * params.kp_tracker.bearing_angle_multiplicator;
end
updateConsole(params,...
              sprintf('  Bearing angle interval: [%0.2f. %0.2f]\n',...
              bearing_angle_low_thr, bearing_angle_up_thr));
          
% create idx vector of trianguable candidate points
idx_good_trianguable = ((bearing_angle_deg > bearing_angle_low_thr)...
    & (kp_tracks_updated.nr_trackings >= params.kp_tracker.min_nr_trackings)...
    & (bearing_angle_deg < bearing_angle_up_thr));

p_candidates_first = kp_tracks_updated.first_obs_kp(:,idx_good_trianguable);
p_candidates_first_pose = kp_tracks_updated.first_obs_pose(:,idx_good_trianguable);
p_candidates_j = kp_tracks_updated.candidate_kp(:,idx_good_trianguable);
p_candidates_j_nr_tracking = kp_tracks_updated.nr_trackings(idx_good_trianguable);

% show matches from first and j image of keypoints
if (params.cont.figures && params.kp_tracker.show_matches)
    figure(fig_kp_tracks);
    if (size(p_candidates_first,2) > 0) % 0 in first frame
        plotPoints(p_candidates_first(1:2,:),'gx');
        plotPoints(p_candidates_j(1:2,:),'bx');
        plotMatches(1:size(p_candidates_j,2),p_candidates_j,p_candidates_first,'g-');     
    end
    title('Previous kp (red), New kp (yellow), Trianguable First kp (gx), Trianguable New kp (bx)');
end

p_hom_candidates_first_uv = [flipud(p_candidates_first); ones(1,size(p_candidates_first,2))];
p_hom_candidates_j_uv = [flipud(p_candidates_j); ones(1,size(p_candidates_j,2))];

% variable initialization
Cfirst_P_hom_new = zeros(4,size(p_candidates_first,2));
Cj_P_hom_new = zeros(4,size(p_candidates_first,2));

updateConsole(params,...
              sprintf('  Number of trianguable keypoint candidates: %i\n',...
              nnz(idx_good_trianguable)));

    for i=1:size(p_candidates_first,2)
        T_WCfirst = reshape(p_candidates_first_pose(:,i), [4,4]);

        % calculate M's
        M_Cfirst = K * eye(3,4);
        T_CjW = tf2invtf(T_WCj);
        T_CjCfirst = T_CjW * T_WCfirst;
        M_CjCfirst = K * T_CjCfirst(1:3,:);

        % triangulate landmark    
        Cfirst_P_hom_new(:,i) = linearTriangulation(p_hom_candidates_first_uv(:,i),p_hom_candidates_j_uv(:,i),M_Cfirst,M_CjCfirst);
        Cj_P_hom_new(:,i) = T_CjCfirst*Cfirst_P_hom_new(:,i);
    end

%% Update keypoint tracks, Cj_landmarks and p_new_matched_triang
% delete candidate keypoint used for triangulation from updated_kp_tracks
kp_tracks_updated.candidate_kp = kp_tracks_updated.candidate_kp(:,~idx_good_trianguable);
kp_tracks_updated.first_obs_kp = kp_tracks_updated.first_obs_kp(:,~idx_good_trianguable);
kp_tracks_updated.first_obs_pose = kp_tracks_updated.first_obs_pose(:,~idx_good_trianguable);
kp_tracks_updated.nr_trackings = kp_tracks_updated.nr_trackings(~idx_good_trianguable);

%% Filter landmarks with spherical and reprojection filter
Cj_reprojected_points_uv = [];
Cj_P_hom_new_inliers = [];
p_candidates_j_inliers = [];
p_candidates_j_inliers_nr_tracking = [];
p_candidates_first_inliers = [];

if size(Cj_P_hom_new,2) > 0 
    [Cj_P_hom_new_neigb, outFOV_idx] = applySphericalFilter(params, Cj_P_hom_new, params.cont.landmarks_cutoff);
    %fprintf('  -> Apply spherical filter to new Landmarks: (%i/%i) inside Sphere\n',size(Cj_P_hom_new_neigb, 2) , size(outFOV_idx, 2)+size(Cj_P_hom_new_neigb, 2));

    % remove unrealistic landmarks and corresponding keypoints
    if (size(outFOV_idx)<size(Cj_P_hom_new,2)) % some Landmarks were in neigbourhood
        Cj_P_hom_new = Cj_P_hom_new_neigb;
        p_candidates_first(:,outFOV_idx)=[];
        p_candidates_j(:,outFOV_idx)=[];
        p_candidates_first_pose(:,outFOV_idx)=[];
        p_candidates_j_nr_tracking(:,outFOV_idx)=[];

        % reproject realistic landmarks to remove outliers
        Cj_reprojected_points_uv = projectPoints(Cj_P_hom_new(1:3,:), K);
        difference = p_candidates_j - flipud(Cj_reprojected_points_uv);
        errors = sum(difference.^2, 1);
        reproj_inliers = errors < params.kp_tracker.max_reproj_error^2;

        Cj_P_hom_new_inliers = Cj_P_hom_new(:, reproj_inliers);
        p_candidates_first_inliers = p_candidates_first(:, reproj_inliers);
        p_candidates_j_inliers = p_candidates_j(:, reproj_inliers);
        p_candidates_j_inliers_nr_tracking = p_candidates_j_nr_tracking(reproj_inliers);
        assert(size(Cj_P_hom_new_inliers,2) == size(p_candidates_j_inliers,2));

        updateConsole(params,...
                      sprintf('  Removed %i of %i realistic landmarks due to too big reprojection error\n',...
                      nnz(~reproj_inliers), size(Cj_P_hom_new,2)));

    else
        updateConsole(params, '  None of the triangulated landmarks were realistic!\n');
        p_candidates_j = [];
    end

    % display triangulated backprojected keypoints
    if (params.cont.figures && params.kp_tracker.show_triangulated)
        good_idx_match = 1:size(Cj_reprojected_points_uv,2);
        figure(fig_kp_triangulate);
        if (size(p_candidates_j,2) > 0) % 0 in first frame
            plotPoints(p_candidates_j,'r.');
            plotPoints(flipud(Cj_reprojected_points_uv),'gx');
            plotMatches(good_idx_match,p_candidates_j,flipud(Cj_reprojected_points_uv),'m-');
            % plotPoints(projected_points,'g.');
        end
        title('Trianguable query kp (red), reprojected generated landmarks (green)');
        hold off;

        if size(Cj_P_hom_new_inliers,2) > 0
        % plots individual cams of landmarks and the landmarks themself
            if params.cont.plot_new_landmarks && params.cont.figures
                figure
                hold on
                cam_poses = unique(p_candidates_first_pose', 'rows');
                cam_poses = cam_poses';
                T_CjCfirsts = zeros(4,4,size(cam_poses, 2));  

                for i=1:size(cam_poses, 2)
                    T_WCfirst = reshape(cam_poses(:,i), [4,4]);
                    T_CjCfirsts(:,:,i) = T_CjW*T_WCfirst;
                end

                plotLandmarks(Cj_P_hom_new_inliers(1:3,:), 'y', 'down')
                plotCam(T_CjCfirsts, 0.2, 'red');
                plotCam(eye(3,4), 0.2, 'black');
            end
        end
    end
end

end
