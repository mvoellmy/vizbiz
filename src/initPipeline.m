function [I_init, keypoints_init, landmarks_init] = initPipeline(params, I_i1, I_i2, K)
% Returns initialization image and corresponding keypoints and landmarks
% after checking for valid correspondences between a bootstrap image pair.
% Optionally, precalculated outputs are loaded.
% 
% Input:
%  - params(struct) : parameter struct
%  - I_i1(size) : first image
%  - I_i2(size) : second image
%
% Output:
%  - I_init(size) : initialization image
%  - keypoints_init(2xN) : matched keypoints from image pair, each [v,u]
%  - landmarks_init(3xN) : common triangulated 3D points

if params.init.use_KITTI_precalculated_init % todo: still needed?
    % assign second image as initialization image
    I_init = I_i2;
    
    % load precalculated keypoints and landmarks
    keypoints_init = load('../datasets/kitti/precalculated/keypoints.txt')';
    landmarks_init = load('../datasets/kitti/precalculated/landmarks.txt')';
else
    % assign second image as initialization image
    I_init = I_i2;

    % find 2D correspondences
    [p_i1,p_i2,~] = findCorrespondeces(params,I_i1,I_i2);
    
    % homogenize points
    p_hom_i1 = [p_i1; ones(1,length(p_i1))];
    p_hom_i2 = [p_i2; ones(1,length(p_i2))];    

    % estimate the essential matrix E using normalized 8-point algorithm
    % and RANSAC for outlier rejection
    %E = estimateEssentialMatrix(p_hom_i1,p_hom_i2,K,K); % todo: remove
    E = eightPointRansac(params,p_hom_i1,p_hom_i2,K,K);

    % extract the relative camera pose (R,t) from the essential matrix
    [Rots,u3] = decomposeEssentialMatrix(E);

    % disambiguate among the four possible configurations
    [C2_R_C2C1,C2_t_C2C1] = disambiguateRelativePose(Rots,u3,p_hom_i1,p_hom_i2,K,K);
    
    % world reference (first frame)
    W_T_WC1 = [eye(3,3), zeros(3,1);
               zeros(1,3),       1];

    % construct C2 to W transformation
    C2_T_C2C1 = [C2_R_C2C1,  C2_t_C2C1;
                 zeros(1,3),       1];
    C1_T_C1C2 = [C2_R_C2C1',  -C2_R_C2C1'*C2_t_C2C1;
                 zeros(1,3),       1];
    W_T_WC2 = W_T_WC1 * C1_T_C1C2;

    % feature: Refine pose with BA
    % TODO

    % triangulate a point cloud using the final transformation (R,T)
    M1 = K*W_T_WC1(1:3,:);
    M2 = K*C2_T_C2C1(1:3,:); %M2 = K*W_T_WC2(1:3,:);
    P_hom_init = linearTriangulation(p_hom_i1,p_hom_i2,M1,M2); % todo: VERIFY landmarks must be in world frame!
    
    % remove landmarks with negative Z coordinate % todo: dedicate function
    % with cyclindrical cutoff? and display amount of dropped landmarks?
    outFOV_idx = find(P_hom_init(3,:) <0 );
    P_hom_init(:,outFOV_idx) = [];
    
    % remove corresponding keypoints
    p_i2(:,outFOV_idx) = [];
    
    % feature: non-linear refinement with minimizing 
    % Sum of Squared Reprojection Errors
    % TODO
    
    % assign initialization entities
    keypoints_init = flipud(p_i2);
    landmarks_init = P_hom_init(1:3,:);
    
    % todo: display summary
    % about number of keypoints, landmarks, 
end

% check for same number of keypoints and landmarks
assert(size(keypoints_init,2) == size(landmarks_init,2));

% display initialization landmarks and bootstrap motion
if (params.init.show_landmarks && ~params.init.use_KITTI_precalculated_init)
    figure('name','Landmarks and motion of bootstrap image pair');
    plot3DFrustrum(W_T_WC1,5); % todo: increase scale internally
    plot3DFrustrum(W_T_WC2,5);
    plotLandmarks(landmarks_init);
end

end
