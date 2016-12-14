function [ P_refined, T_refined ] = bundleAdjust(P, p, T, K, fixed_cams )
% Wraps our conventionally used parameters to be used with the Bundle
% Adjust function of Matlab. See implementation in initPipeline.m
% Attention! Inputs are NOT homogenized coordinates.
% 
% Input:
%  - P(3xN)     : List of 3D Points in world-frame
%  - p(nC*2xN)  : Matrix containing 2D Points sorted according to
%  their correspondance with each other and the 3D points.
%  - T(nC*4x4)  : Stack of transformation matrices towards the individual
%  cameras
%  - K(3x3) : intrinsics matrix of camera
%  - fixed_cams(1xC) : Vector with camera_ids defining which cams are fixed
%  in world
%
% Output:
%  - P_refined(3xN) : Bundle adjusted 3D Points in world-frame
%  - T_refined(nC*4x4) : refined transformation matrices of cameras
%
% Definitions:
%  - nC(int) : number of cameras

% Potential Improvements:
%  Preallocate point_tracks
%  Remove for loops where possible

fprintf('  bundle adjust points...\n')

nr_of_cams = size(p, 1)/2;
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

for i=1:4:4*nr_of_cams % todo: might be possible to remove for loop
    orientations((i-1)/4 + 1) = {T(i:i+2, 1:3)};
    locations((i-1)/4 + 1) = {T(i:i+2, 4)'};
end

cameraPoses = table;
cameraPoses.ViewId = uint32(view_ids');
cameraPoses.Orientation = orientations;
cameraPoses.Location = locations;
cameraParams = cameraParameters('IntrinsicMatrix', K');

[P_refined, refinedPoses] = bundleAdjustment(P', point_tracks, cameraPoses, cameraParams, 'FixedViewIDs', fixed_cams );

P_refined = P_refined';
T_refined = zeros(size(T));

for i=1:nr_of_cams % todo: pretty sure this can be indexed nicer and potentially done without a for loop
    T_refined(1+(i-1)*4:4+(i-1)*4,1:4) = [cell2mat(refinedPoses.Orientation(i)), cell2mat(refinedPoses.Location(i))';
           zeros(1,3),       1];
end
    
end
