function [I_init, keypoints_init, landmarks_init] = initPipeline(p, I_i1, I_i2,K)
% Returns initialization image and corresponding keypoints and landmarks
% after checking for valid correspondences between a bootstrap image pair.
% Optionally, precalculated outputs are loaded.
% 
% Input:
%  - p : parameter struct
%  - I_i0(size) : first image
%  - I_i1(size) : second image
%
% Output:
%  - I_init(size) : initialization image
%  - keypoints_init(2xN) : ...
%  - landmarks_init(3xM) : common triangulated 3D points

if p.init.use_KITTI_precalculated_init
    % assign second image as initialization image
    I_init = I_i2;
    
    % load precalculated keypoints and landmarks
    keypoints_init = load('../datasets/kitti/precalculated/keypoints.txt')';
    landmarks_init = load('../datasets/kitti/precalculated/landmarks.txt')'; 
    keypoints_init = [keypoints_init;ones(1,length(keypoints_init))];
    
else
    % assign second image as initialization image
    I_init = I_i2;

    % Find 2D correspondences
    [p_i1_raw,p_i2_raw,matches] = findCorrespondeces(p, I_i1, I_i2); % 2D points
    
    % Sort points
    p_i1 = p_i1_raw(:,matches>0);
    p_i2 = p_i2_raw(:,matches); % sorted points matched with p_i1
   
    % Homogenize points
    p_i1_h = [p_i1;ones(1,length(p_i1))];
    p_i2_h = [p_i2;ones(1,length(p_i2))];
    
    keypoints_init = p_i2_h;

    % Estimate the essential matrix E using the 8-point algorithm
    E = estimateEssentialMatrix(p_i1_h, p_i2_h, K, K);

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

% display initialization image
if p.show_init_images
    figure('name','Initialization image');
    imshow(I_init);
    hold on;
    plot(keypoints_init(2,:), keypoints_init(1,:), 'rx', 'Linewidth', 1);
    plotMatches(matches, p_i1_raw, p_i2_raw);
end

end
