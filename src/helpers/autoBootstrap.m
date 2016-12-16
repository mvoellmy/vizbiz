function [img0, img1, bootstrap_frame_1_idx, bootstrap_frame_2_idx] = autoBootstrap(params, K)
% Returns bootstrap image pair and corresponding frame indices.
% 
% Input:
%  - params(struct) : parameter struct
%  - K(3x3) : camera calibration matrix
%
% Output:
%  - img0(size) : first bootstrap image
%  - img1(size) : second bootstrap image
%  - bootstrap_frame_1_idx(1x1) : dataset image index of img0
%  - bootstrap_frame_2_idx(1x1) : dataset image index of img1

global fig_boot;

if params.boot.show_bootstrap_images
    fig_boot = figure('name','Bootstrapping');
end

if params.auto_bootstrap
    bootstrap_frame_1_idx = 1;

    if params.ds == 0
        img0 = imread([params.kitti_path '/00/image_0/' ...
            sprintf('%06d.png',bootstrap_frame_1_idx)]);
    elseif params.ds == 1
        images = dir([params.malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
        left_images = images(3:2:end);
        img0 = rgb2gray(imread([params.malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(bootstrap_frame_1_idx).name]));
    elseif params.ds == 2
        img0 = rgb2gray(imread([params.parking_path ...
            sprintf('/images/img_%05d.png',bootstrap_frame_1_idx)]));
    else
        assert(false);
    end

    % search for second bootstrapping image
    bootstrap_pair_found = false;
    candidate_frame_idx = bootstrap_frame_1_idx;

    while ~bootstrap_pair_found    
        candidate_frame_idx = candidate_frame_idx + 1;
        
        % read in candidate image
        img_candidate = currentFrame(params, candidate_frame_idx);

        % find 2D correspondences (sorted)
        [p_i1,p_i2] = findCorrespondeces_boot(params,img0,img_candidate);

        % homogenize points
        p_hom_i1 = [p_i1; ones(1,length(p_i1))];
        p_hom_i2 = [p_i2; ones(1,length(p_i2))];    

        % estimate the essential matrix E using normalized 8-point algorithm
        % and RANSAC for outlier rejection
        E = eightPointRansac(params,p_hom_i1,p_hom_i2,K,K);

        % extract the relative camera pose (R,t) from the essential matrix
        [Rots,u3] = decomposeEssentialMatrix(E);

        % disambiguate among the four possible configurations
        [R_C2C1,C2_t_C2C1] = disambiguateRelativePose(Rots,u3,p_hom_i1,p_hom_i2,K,K);

        % construct C2 to W transformation
        T_C2C1 = [R_C2C1,  C2_t_C2C1;
                  zeros(1,3),      1];
        T_C1C1 = eye(4,4);        
        
        % extract baseline distance
        T_C1C2 = tform2invtform(T_C2C1);
        T_C1C2(1:3,4)'
        C1_baseline = norm(T_C1C2(1:3,4)')

        % triangulate a point cloud using the final transformation (R,t)
        M1 = K*T_C1C1(1:3,:);
        M2 = K*T_C2C1(1:3,:);
        C1_P_hom_init = linearTriangulation(p_hom_i1, p_hom_i2, M1, M2);
        
        % discard landmarks not contained in cylindrical neighborhood
        [C1_P_hom_init, ~] = applyCylindricalFilter(C1_P_hom_init, params.boot.landmarks_cutoff);
        
        % todo: apply non-negative filter
        
        % calculate average depth of triangulated points
        C1_avg_depth = mean(C1_P_hom_init(3,:));
        fprintf('  Baseline estimate of frame pair (%i,%i): %0.4f\n',...
                bootstrap_frame_1_idx,candidate_frame_idx,C1_baseline);
        
        % decide wether candidate is suited as bootstrap image
        if candidate_frame_idx == 10%(C1_baseline/C1_avg_depth >= 0.05)
           img1 = img_candidate;
           bootstrap_frame_2_idx = candidate_frame_idx;
           bootstrap_pair_found = true;
        end
        fprintf('\n');
    end

else
    bootstrap_frame_1_idx = 1;
    bootstrap_frame_2_idx = bootstrapFrames(params.ds,'second');
    
    if params.ds == 0
        img0 = imread([params.kitti_path '/00/image_0/' ...
            sprintf('%06d.png',bootstrap_frame_1_idx)]);
        img1 = imread([params.kitti_path '/00/image_0/' ...
            sprintf('%06d.png',bootstrap_frame_2_idx)]);
    elseif params.ds == 1
        images = dir([params.malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
        left_images = images(3:2:end);
        img0 = rgb2gray(imread([params.malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(bootstrap_frame_1_idx).name]));
        img1 = rgb2gray(imread([params.malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(bootstrap_frame_2_idx).name]));
    elseif params.ds == 2
        img0 = rgb2gray(imread([params.parking_path ...
            sprintf('/images/img_%05d.png',bootstrap_frame_1_idx)]));
        img1 = rgb2gray(imread([params.parking_path ...
            sprintf('/images/img_%05d.png',bootstrap_frame_2_idx)]));
    else
        assert(false);
    end
        
    figure(fig_boot);
    subplot(2,1,1);
    imshow(img0);
    axis equal;
    title('Bootstrap frame 1');
    
    subplot(2,1,2);
    imshow(img1);
    axis equal;
    title('Bootstrap frame 2');
end

% display frames chosen
fprintf(['  Bootstrap image 1 index: %i\n',...
         '  Bootstrap image 2 index: %i\n'],...
         bootstrap_frame_1_idx, bootstrap_frame_2_idx);

end
