function [ W_P_refined, T_WC_refined ] = bundleAdjust(params, W_P, p, T_WC, K, fixed_cams )
% Wraps our conventionally used parameters to be used with the Bundle
% Adjust function of Matlab. See implementation in initPipeline.m
% Attention! Inputs are NOT homogenized coordinates.
% 
% Input:
%  - params(struct) : parameter struct
%  - W_P(3xN)     : list of 3D Points in world-frame
%  - p(nC*2xN)  : matrix containing 2D Points sorted according to
%  their correspondance with each other and the 3D points. [u, v]
%  - T_WC(4x4xnC)  : stack of transformation matrices towards the individual
%  cameras
%  - K(3x3) : intrinsics matrix of camera
%  - fixed_cams(1xC) : vector with camera_ids defining which cams are fixed
%  in world
%
% Output:
%  - W_P_refined(3xN) : bundle adjusted 3D points in world-frame
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
orientations = cell(nr_of_cams, 1);
locations = cell(nr_of_cams, 1);

for i=1:nr_of_cams % todo: might be possible to remove for loop
    orientations(i) = {T_WC(1:3, 1:3, i)'}; % This is transposed as we want the orientation of the cam in the world frame
    locations(i) = {T_WC(1:3, 4, i)'};
end

cameraPoses = table;
cameraPoses.ViewId = uint32(view_ids');
cameraPoses.Orientation = orientations;
cameraPoses.Location = locations;
cameraParams = cameraParameters('IntrinsicMatrix', K');

[W_P_refined, refinedPoses] = bundleAdjustment(W_P', point_tracks, cameraPoses, cameraParams, 'FixedViewIDs', fixed_cams); %, 'PointsUndistorted', true

W_P_refined = W_P_refined';

T_WC_refined(1:3,1:3,1:nr_of_cams) = reshape(cell2mat(refinedPoses.Orientation(1:nr_of_cams))',3,3,nr_of_cams);
T_WC_refined(1:3,4,1:nr_of_cams) = reshape(cell2mat(refinedPoses.Location(1:nr_of_cams))',1,3,nr_of_cams);
T_WC_refined(4,1:3,1:nr_of_cams) = zeros(1,3,nr_of_cams);
T_WC_refined(4,4,1:nr_of_cams) = ones(1,1,nr_of_cams);
    
end
