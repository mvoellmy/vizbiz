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
%  - keypoints_init(2xN) : matched keypoints from boostrap image pair
%  - landmarks_init(3xN) : common triangulated 3D points

if params.init.use_KITTI_precalculated_init
    % assign second image as initialization image
    I_init = I_i2;
    
    % load precalculated keypoints and landmarks
    keypoints_init = load('../datasets/kitti/precalculated/keypoints.txt')';
    landmarks_init = load('../datasets/kitti/precalculated/landmarks.txt')';
else
    % assign second image as initialization image
    I_init = I_i2;

    % find 2D correspondences
    [p_i1,p_i2,matches] = findCorrespondeces(params,I_i1,I_i2);

    % assign initialization keypoints
    keypoints_init = p_i2;
    
    % homogenize points
    p_hom_i1 = [p_i1; ones(1,length(p_i1))];
    p_hom_i2 = [p_i2; ones(1,length(p_i2))];    

    % estimate the essential matrix E using normalized 8-point algorithm
    E = estimateEssentialMatrix(p_hom_i1,p_hom_i2,K,K);

    % extract the relative camera positions (R,T) from the essential matrix
    % obtain extrinsic parameters (R,t) from E
    [Rots,u3] = decomposeEssentialMatrix(E);

    % disambiguate among the four possible configurations
    [R_C2_W,T_C2_W] = disambiguateRelativePose(Rots,u3,p_hom_i1,p_hom_i2,K,K);

    % feature: Refine pose with BA
    % TODO

    % triangulate a point cloud using the final transformation (R,T)
    M1 = K * eye(3,4);
    M2 = K * [R_C2_W, T_C2_W];
    P_hom_init = linearTriangulation(p_hom_i1,p_hom_i2,M1,M2); % VERIFY landmarks must be in world frame!
    
    landmarks_init = P_hom_init(1:3,:);
end

% check for same number of keypoints and landmarks
assert(size(keypoints_init,2) == size(landmarks_init,2));

% display initialization image with keypoints and matches
if params.show_init_landmarks
    figure('name','Initialization points/landmarks');
    subplot(1,2,1);
    imshow(I_init);
    hold on;    
    plotPoints(keypoints_init);    
    
    if ~params.init.use_KITTI_precalculated_init
        subplot(1,2,2);
        plotLandmarks(landmarks_init);
    end
end

end
