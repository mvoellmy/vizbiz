function [ W_P_refined, T_WC_refined ] = bundleAdjust(W_P, p, T_WC, K, fixed_cams )
% Wraps our conventionally used parameters to be used with the Bundle
% Adjust function of Matlab. See implementation in initPipeline.m
% Attention! Inputs are NOT homogenized coordinates.
% 
% Input:
%  - P(3xN)     : list of 3D Points in world-frame
%  - p(nC*2xN)  : matrix containing 2D Points sorted according to
%  their correspondance with each other and the 3D points.

%  - T_WC(4x4xnC)  : Stack of transformation matrices from the cams to
%  world
%  - fixed_cams(1xC) : Vector with camera_ids defining which cams are fixed
%  in world
%
% Output:
%  - W_P_refined(3xN) : Bundle adjusted 3D Points in world-frame
%  - T_WC_refined(4x4xnC) : refined transformation matrices of cameras

%
% Definitions:
%  - nC(int) : number of cameras

% Potential Improvements:
%  Preallocate point_tracks
%  Remove for loops where possible

    fprintf('  bundle adjust points...\n')

    for i=1:nr_of_keypoints % todo: might be possible to remove for loop
        p_corresponding = vec2mat(p(:,i),2);
        point_tracks(i) = pointTrack(view_ids, p_corresponding);
    end
    
    % fill cameraPoses vectors
    orientations = cell(nr_of_cams, 1);
    locations = cell(nr_of_cams, 1);
    
    for i=1:nr_of_cams % todo: might be possible to remove for loop
        orientations(i) = {T_WC(1:3, 1:3, i)'}; % This is transposed as we want the orientation from the cam in the world
        locations(i) = {T_WC(1:3, 4, i)'};
    end
    
    cameraPoses = table;
    cameraPoses.ViewId = uint32(view_ids');
    cameraPoses.Orientation = orientations;
    cameraPoses.Location = locations;
    cameraParams = cameraParameters('IntrinsicMatrix', K');
    
    [W_P_refined, refinedPoses] = bundleAdjustment(W_P', point_tracks, cameraPoses, cameraParams, 'FixedViewIDs', fixed_cams );
    
    W_P_refined = W_P_refined';
    T_WC_refined = zeros(size(T_WC));
    
    for i=1:nr_of_cams % todo: pretty sure this can be indexed nicer and potentially done without a for loop
        T_WC_refined(1:4, 1:4, i) = [cell2mat(refinedPoses.Orientation(i))', cell2mat(refinedPoses.Location(i))';
                                  zeros(1,3),       1];
    end

    
end
