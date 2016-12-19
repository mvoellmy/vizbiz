function [I_init, keypoints_init, C2_landmarks_init, T_C1C2] = initPipeline(params, I_i1, I_i2, K, T_WC1)
% Returns initialization image and corresponding sorted keypoints and landmarks
% after checking for valid correspondences between a bootstrap image pair.
% Optionally, precalculated outputs are loaded.
% 
% Input:
%  - params(struct) : parameter struct
%  - I_i1(size) : first image
%  - I_i2(size) : second image
%  - K(3x3) : camera calibration matrix
%  - T_WC1(4x4) : fixed transformation from C1 to W
%
% Output:
%  - I_init(size) : initialization image
%  - keypoints_init(2xN) : matched keypoints from image pair, each [v,u]
%  - C2_landmarks_init(3xN) : C2-referenced triangulated 3D points
%  - T_C1C2(4x4) : homogeneous transformation matrix C2 to C1

if params.init.use_KITTI_precalculated_init % todo: still needed?
    % assign second image as initialization image
    I_init = I_i2;
    
    % load precalculated keypoints and landmarks
    keypoints_init = load('../datasets/kitti/precalculated/keypoints.txt')';
    C2_landmarks_init = load('../datasets/kitti/precalculated/landmarks.txt')';
    
    T_C1C2 = eye(4);
else
    % assign second image as initialization image
    I_init = I_i2;

    % find 2D correspondences (sorted)
    [p_i1,p_i2] = findCorrespondeces(params,I_i1,I_i2);
    
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
    
    % construct transformation
    T_C2C1 = [R_C2C1,  C2_t_C2C1;
              zeros(1,3),      1];
    T_C1C2 = tform2invtform(T_C2C1);
    T_C1C1 = eye(4,4);
    T_WC2 = T_WC1*T_C1C2;
    
    % triangulate a point cloud using the final transformation (R,T)
    M1 = K*T_C1C1(1:3,:);
    M2 = K*T_C2C1(1:3,:);
    C1_P_hom_init = linearTriangulation(p_hom_i1, p_hom_i2, M1, M2);

    % discard landmarks not contained in cylindrical neighborhood
    [C1_P_hom_init, outFOV_idx] = applyCylindricalFilter(C1_P_hom_init, params.init.landmarks_cutoff);

    % remove corresponding keypoints
    p_i2(:,outFOV_idx) = [];
    p_hom_i1(:,outFOV_idx) = [];
    p_hom_i2(:,outFOV_idx) = [];
    W_P_hom_init = T_WC1*C1_P_hom_init;

    if params.init.use_BA
        if params.init.show_BA_comp
            figure('name','BundleaAdjustment Comparison');
            subplot(1,2,1)
            plotLandmarks(W_P_hom_init(1:3,:),'z','up');
            hold on
            plotCam(T_WC1,2,'black');
            plotCam(T_WC2,2,'red');
            title('Before BA');
        end
        
        [W_P_init, T_refined] = bundleAdjust(W_P_hom_init(1:3,:), [p_hom_i1(1:2,:); p_hom_i2(1:2,:)], cat(3, T_WC1, T_WC2), K, 1);   
        W_P_hom_init(1:3,:) = W_P_init;
        
        % update homogeneous transformations
        T_WC1 = T_refined(1:4, 1:4, 1);
        T_WC2 = T_refined(1:4, 1:4, 2);
        T_C1W = tform2invtform(T_WC1);
        T_C1C2 = T_C1W*T_WC2;
        T_C2C1 = tform2invtform(T_C1C2);

        C1_P_hom_init = T_C1W*W_P_hom_init;
        
        % discard landmarks not contained in cylindrical neighborhood
        [C1_P_hom_init, outFOV_idx] = applyCylindricalFilter(C1_P_hom_init, params.init.landmarks_cutoff);
        W_P_hom_init = T_WC1*C1_P_hom_init;

        % remove corresponding keypoints
        p_i2(:,outFOV_idx) = [];
       
        if params.init.show_BA_comp
            subplot(1,2,2)
            plotLandmarks(W_P_hom_init(1:3,:),'z','up');
            hold on
            plotCam(T_WC1,2,'black');
            plotCam(T_WC2,2,'red');
            title('After BA');
        end
    end
    
    % assign initialization entities
    keypoints_init = flipud(p_i2);
    C2_P_hom_init = T_C2C1*C1_P_hom_init;
    C2_landmarks_init = C2_P_hom_init(1:3,:);

    
    
    % display statistics
    % todo: extend with baseline length,...
    fprintf(['  Number of initialization keypoints: %i\n',...
             '  Number of initialization landmarks: %i\n'],...
             size(keypoints_init,2), size(C2_landmarks_init,2));
end

% check for same number of keypoints and landmarks
assert(size(keypoints_init,2) == size(C2_landmarks_init,2));

end
