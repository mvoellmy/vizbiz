function [img1, img2, bootstrap_frame_1_idx, bootstrap_frame_2_idx] = getBootstrapFrames(params, K)
% Returns bootstrap image pair and corresponding frame indices.
% 
% Input:
%  - params(struct) : parameter struct
%  - K(3x3) : camera calibration matrix
%
% Output:
%  - img1(size) : first bootstrap image
%  - img2(size) : second bootstrap image
%  - bootstrap_frame_1_idx(1x1) : dataset image index of img0
%  - bootstrap_frame_2_idx(1x1) : dataset image index of img1

global fig_boot gui_handles;

if (params.boot.figures && params.boot.show_boot_images)
    fig_boot = figure('name','Bootstrapping');
end

if params.auto_bootstrap
    bootstrap_frame_1_idx = 1;

    if params.ds == 0
        img1 = imread([params.kitti_path '/00/image_0/' ...
            sprintf('%06d.png',bootstrap_frame_1_idx)]);
    elseif params.ds == 1
        images = dir([params.malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
        left_images = images(3:2:end);
        img1 = rgb2gray(imread([params.malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(bootstrap_frame_1_idx).name]));
    elseif params.ds == 2
        img1 = rgb2gray(imread([params.parking_path ...
            sprintf('/images/img_%05d.png',bootstrap_frame_1_idx)]));
    else
        assert(false);
    end
    
    % compute harris scores for database image
    database_harris = harris(img1, params.corr.harris_patch_size, params.corr.harris_kappa);
    % compute keypoints for database image
    database_keypoints = selectKeypoints(database_harris, params.boot.num_keypoints, params.corr.nonmaximum_supression_radius);

    % search for second bootstrapping image
    bootstrap_pair_found = false;
    candidate_frame_idx = bootstrap_frame_1_idx;

    while ~bootstrap_pair_found    
        candidate_frame_idx = candidate_frame_idx + 1;
        
        % read in candidate image
        img_candidate = getFrame(params, candidate_frame_idx);
        
        % update gui image
        if params.through_gui
            gui_updateImage(img_candidate, gui_handles.ax_current_frame);
        end
        
        % find 2D correspondences (sorted)
        [p_i1, p_i2] = findCorrespondeces_boot(params, img1, database_keypoints, img_candidate);

        % homogenize points
        p_hom_i1 = [p_i1; ones(1,length(p_i1))];
        p_hom_i2 = [p_i2; ones(1,length(p_i2))];    

        % estimate the essential matrix E using normalized 8-point algorithm
        % and RANSAC for outlier rejection
        [E, max_num_inliers] = eightPointRansac(params, p_hom_i1, p_hom_i2, K, K);

        % extract the relative camera pose (R,t) from the essential matrix
        [Rots, u3] = decomposeEssentialMatrix(E);

        % disambiguate among the four possible configurations
        [R_C2C1, C2_t_C2C1] = disambiguateRelativePose(Rots, u3, p_hom_i1, p_hom_i2, K, K);

        % construct C2 to W transformation
        T_C2C1 = [R_C2C1,  C2_t_C2C1;
                  zeros(1,3),      1];
        T_C1C1 = eye(4,4);
        
        % extract baseline distance
        T_C1C2 = tform2invtform(T_C2C1);
        C1_baseline = T_C1C2(3,4); % todo: correct? better norm of T_C1C2(1:3,4)?

        % triangulate a point cloud using the final transformation (R,t)
        M1 = K*T_C1C1(1:3,:);
        M2 = K*T_C2C1(1:3,:);
        C1_P_hom_init = linearTriangulation(p_hom_i1, p_hom_i2, M1, M2);
        
        % discard landmarks not contained in cylindrical neighborhood
        [C1_P_hom_init, ~] = applyCylindricalFilter(C1_P_hom_init, params.boot.landmarks_cutoff);
        
        % show bootstrap pair landmarks
        if (params.boot.figures && params.boot.show_boot_landmarks)
            figure('name',sprintf('Landmarks frame pair (%i,%i)', bootstrap_frame_1_idx, candidate_frame_idx));
            plotLandmarks(C1_P_hom_init(1:3,:),'y','down');
        end
        
        % calculate average depth of triangulated points
        C1_avg_depth = mean(C1_P_hom_init(3,:));
        updateConsole(params,...
                      sprintf('  Baseline-Depth ratio of frame pair (%i,%i): %0.1f %%\n',...
                      bootstrap_frame_1_idx, candidate_frame_idx, 100*C1_baseline/C1_avg_depth));
        
        % check for sufficient number of bootstrap inlier matches
        if (max_num_inliers > params.boot.min_num_inlier_kps)
            % decide wether candidate is suited as bootstrap image
            if (C1_baseline/C1_avg_depth >= params.boot.min_b2dratio)
               bootstrap_frame_2_idx = candidate_frame_idx;
               img2 = img_candidate;
               bootstrap_pair_found = true;
            end
        else
            bootstrap_frame_2_idx = candidate_frame_idx - 1;
            img2 = getFrame(params, bootstrap_frame_2_idx);
            bootstrap_pair_found = true;
        end
        updateConsole(params, '\n');
    end

else
    bootstrap_frame_1_idx = 1;
    bootstrap_frame_2_idx = bootstrapFrames(params.ds,'second');
    
    if params.ds == 0
        img1 = imread([params.kitti_path '/00/image_0/' ...
            sprintf('%06d.png',bootstrap_frame_1_idx)]);
        img2 = imread([params.kitti_path '/00/image_0/' ...
            sprintf('%06d.png',bootstrap_frame_2_idx)]);
    elseif params.ds == 1
        images = dir([params.malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
        left_images = images(3:2:end);
        img1 = rgb2gray(imread([params.malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(bootstrap_frame_1_idx).name]));
        img2 = rgb2gray(imread([params.malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(bootstrap_frame_2_idx).name]));
    elseif params.ds == 2
        img1 = rgb2gray(imread([params.parking_path ...
            sprintf('/images/img_%05d.png',bootstrap_frame_1_idx)]));
        img2 = rgb2gray(imread([params.parking_path ...
            sprintf('/images/img_%05d.png',bootstrap_frame_2_idx)]));
    else
        assert(false);
    end
    
    if (params.boot.figures && params.boot.show_boot_images)
        figure(fig_boot);
        subplot(2,1,1);
        imshow(img1);
        axis equal;
        title('Bootstrap frame 1');

        subplot(2,1,2);
        imshow(img2);
        axis equal;
        title('Bootstrap frame 2');
    end
    
    % update gui image
    if params.through_gui
        gui_updateImage(img1, gui_handles.ax_current_frame);
    end
end

% display frames chosen
updateConsole(params,...
    sprintf(['  Bootstrap image 1 index: %i\n',...
             '  Bootstrap image 2 index: %i\n'],...
             bootstrap_frame_1_idx, bootstrap_frame_2_idx));

end
