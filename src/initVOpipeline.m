function [I_init, keypoints_init, landmarks_init] = initVOpipeline(p, I_i0, I_i1)
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
%  - landmarks_init(3xM) : ...

if p.init.use_KITTI_precalculated_init
    % assign second image as initialization image
    I_init = I_i1;
    
    % load precalculated keypoints and landmarks
    keypoints_init = load('../datasets/kitti/precalculated/keypoints.txt');
    landmarks_init = load('../datasets/kitti/precalculated/landmarks.txt');    
else
    % assign second image as initialization image
    I_init = I_i1;

    % Find 2D correspondences
    [p_i0,p_i1] = find_correspondeces(p, I_i0, I_i1);
    keypoints_init = p_i1;

    % Estimate the essential matrix E using the 8-point algorithm
    E = estimateEssentialMatrix(p_i0, p_i1, K, K);

    % Extract the relative camera positions (R,T) from the essential matrix
    % Obtain extrinsic parameters (R,t) from E
    [Rots,u3] = decomposeEssentialMatrix(E);

    % Disambiguate among the four possible configurations
    [R_C2_W,T_C2_W] = disambiguateRelativePose(Rots,u3,p_i0,p_i1,K,K);

    % Feature: Refine pose with BA
    % TODO

    % Triangulate a point cloud using the final transformation (R,T)
    M1 = K * eye(3,4);
    M2 = K * [R_C2_W, T_C2_W];
    landmarks_init = linearTriangulation(p_i0,p_i1,M1,M2);
end

end
