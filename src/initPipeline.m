function [I_init, keypoints_init, landmarks_init] = initPipeline(params, I_i1, I_i2, K)
% Returns initialization image and corresponding keypoints and landmarks
% after checking for valid correspondences between a bootstrap image pair.
% Optionally, precalculated outputs are loaded.
% 
% Input:
%  - p : parameter struct
%  - I_i1(size) : first image
%  - I_i2(size) : second image
%
% Output:
%  - I_init(size) : initialization image
%  - keypoints_init(Nx2) : matched keypoints from boostrap image pair
%  - landmarks_init(Nx3) : common triangulated 3D points

if params.init.use_KITTI_precalculated_init
    % assign second image as initialization image
    I_init = I_i2;
    
    % load precalculated keypoints and landmarks
    keypoints_init = load('../datasets/kitti/precalculated/keypoints.txt');
    keypoints_init = fliplr(keypoints_init);
    landmarks_init = load('../datasets/kitti/precalculated/landmarks.txt');
else
    % assign second image as initialization image
    I_init = I_i2;

    % Find 2D correspondences
    [p_i1_raw,p_i2_raw,matches] = findCorrespondeces(params,I_i1,I_i2); % 2D points
    
    % Sort points
    p_i1 = p_i1_raw(:,matches>0);
    p_i2 = p_i2_raw(:,matches); % sorted points matched with p_i1
   
    % Homogenize points
    p_i1_h = [p_i1;ones(1,length(p_i1))];
    p_i2_h = [p_i2;ones(1,length(p_i2))];
    
    keypoints_init = p_i2_h;

    % Estimate the essential matrix E using the 8-point algorithm
    E = estimateEssentialMatrix(p_i1_h,p_i2_h,K,K);

    % Extract the relative camera positions (R,T) from the essential matrix
    % Obtain extrinsic parameters (R,t) from E
    [Rots,u3] = decomposeEssentialMatrix(E);

    % Disambiguate among the four possible configurations
    [R_C2_W,T_C2_W] = disambiguateRelativePose(Rots,u3,p_i1_h,p_i2_h,K,K);

    % Feature: Refine pose with BA
    % TODO

    % Triangulate a point cloud using the final transformation (R,T)
    M1 = K * eye(3,4);
    M2 = K * [R_C2_W, T_C2_W];
    landmarks_init = linearTriangulation(p_i1_h,p_i2_h,M1,M2); % VERIFY landmarks must be in world frame!    
end

% check for same number of keypoints and landmarks
assert(size(keypoints_init,1) == size(landmarks_init,1));

% display initialization image with keypoints and matches
if params.show_init_image
    figure('name','Initialization image');
    imshow(I_init);
    hold on;
    plotPoints(keypoints_init);
    if ~params.init.use_KITTI_precalculated_init
        plotMatches(matches,p_i1_raw,p_i2_raw);
    end
end

end
