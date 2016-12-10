function [I_init, keypoints_init, C1_landmarks_init, T_C1C2] = initPipeline(params, I_i1, I_i2, K)
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
%  - C1_landmarks_init(3xN) : C1-referenced triangulated 3D points
%  - T_C1C2(4x4) : homogenious transformationn matrix C2 to C1

if params.init.use_KITTI_precalculated_init % todo: still needed?
    % assign second image as initialization image
    I_init = I_i2;
    
    % load precalculated keypoints and landmarks
    keypoints_init = load('../datasets/kitti/precalculated/keypoints.txt')';
    C1_landmarks_init = load('../datasets/kitti/precalculated/landmarks.txt')';
    
    T_C1C2 = eye(4);
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
    E = eightPointRansac(params,p_hom_i1,p_hom_i2,K,K);

    % extract the relative camera pose (R,t) from the essential matrix
    [Rots,u3] = decomposeEssentialMatrix(E);

    % disambiguate among the four possible configurations
    [R_C2C1,C2_t_C2C1] = disambiguateRelativePose(Rots,u3,p_hom_i1,p_hom_i2,K,K);
    
    % construct C2 to W transformation
    T_C2C1 = [R_C2C1,  C2_t_C2C1;
              zeros(1,3),      1];
    T_C1C2 = [R_C2C1',  -R_C2C1'*C2_t_C2C1;
              zeros(1,3),                1];

    % feature: Refine pose with BA
    % TODO

    % triangulate a point cloud using the final transformation (R,T)
    M1 = K*eye(3,4);
    M2 = K*T_C2C1(1:3,:); %M2 = K*T_C1C2(1:3,:);
    C1_P_hom_init = linearTriangulation(p_hom_i1,p_hom_i2,M1,M2);
    
    % remove landmarks with negative Z coordinate % todo: dedicate function
    % with cyclindrical cutoff? and display amount of dropped landmarks?
    outFOV_idx = find(C1_P_hom_init(3,:) <0 );
    C1_P_hom_init(:,outFOV_idx) = [];
    
    % remove corresponding keypoints
    p_i2(:,outFOV_idx) = [];
    
    % feature: non-linear refinement with minimizing 
    % Sum of Squared Reprojection Errors
    % TODO
    
    % assign initialization entities
    keypoints_init = flipud(p_i2);
    C1_landmarks_init = C1_P_hom_init(1:3,:);
    
    % display statistics
    % todo: extend with baseline length,...
    fprintf(['  Number of initialization keypoints: %i\n',...
             '  Number of initialization landmarks: %i\n'],...
             size(keypoints_init,2), size(C1_landmarks_init,2));
end

% check for same number of keypoints and landmarks
assert(size(keypoints_init,2) == size(C1_landmarks_init,2));

end
