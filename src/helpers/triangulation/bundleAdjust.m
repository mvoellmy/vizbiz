function [ W_P_refined, T_WC_refined ] = bundleAdjust(params, W_P, p, T_WC, K, fixed_cams )

% Wraps our conventionally used parameters to be used with the Bundle
% Adjust function of Matlab. See implementation in initPipeline.m
% Attention! Inputs are NOT homogenized coordinates.
% 
% Input:
%  - W_P(3xN)     : list of 3D Points in world-frame
%  - params(struct) : parameter struct
%  - P(3xN)     : list of 3D Points in world-frame
%  - p(nC*2xN)  : matrix containing 2D Points sorted according to
%  their correspondance with each other and the 3D points. [u v]????maybe

%  - T_WC(4x4xnC)  : stack of transformation matrices from the cams to
%  world
%  - fixed_cams(1xC) : vector with camera_ids defining which cams are fixed
%  in world
%
% Output:
%  - W_P_refined(3xN) : bundle adjusted 3D Points in world-frame
%  - T_WC_refined(4x4xnC) : refined transformation matrices of cameras

%
% Definitions:
%  - nC(int) : number of cameras

% Potential Improvements:
%  Preallocate point_tracks
%  Remove for loops where possible

updateConsole(params, '  bundle adjust points...\n');

nr_of_cams = size(T_WC, 3);
nr_of_keypoints = size(p,2);

% vector containing camera_ids
view_ids = 1:nr_of_cams;

for i=1:nr_of_keypoints % todo: might be possible to remove for loop
    p_corresponding = vec2mat(p(:,i),2);
    point_tracks(i) = pointTrack(view_ids, p_corresponding);
end

% fill cameraPoses vectors
orientations_T_CW = cell(nr_of_cams, 1);
locations_T_WC = cell(nr_of_cams, 1);

for i=1:nr_of_cams % todo: might be possible to remove for loop
    orientations_T_CW(i) = {T_WC(1:3, 1:3, i)'}; % This is transposed as we want the orientation of the cam in the world
    locations_T_WC(i) = {T_WC(1:3, 4, i)'}; % why transposed?
end

cameraPoses = table;
cameraPoses.ViewId = uint32(view_ids');
cameraPoses.Orientation = orientations_T_CW;
cameraPoses.Location = locations_T_WC;
cameraParams = cameraParameters('IntrinsicMatrix', K');

[W_P_refined, refinedPoses, reprojectionErrors] = bundleAdjustment(W_P', point_tracks, cameraPoses, cameraParams,...
    'FixedViewIDs', fixed_cams, 'Verbose', true, 'PointsUndistorted', true, 'AbsoluteTolerance', 0.5, 'MaxIterations', 100);

nnz(reprojectionErrors > 5)

W_P_refined = W_P_refined';
T_WC_refined = zeros(size(T_WC));


% maybe its T_CW??
for i=1:nr_of_cams % todo: pretty sure this can be indexed nicer and potentially done without a for loop
    T_WC_refined(1:4, 1:4, i) = [cell2mat(refinedPoses.Orientation(i))', cell2mat(refinedPoses.Location(i))';
                              zeros(1,3),       1];
end

    
end