function [ P_hom_init, W_T_WC1, W_T_WC2 ] = bundleAdjust(P_hom_init, p_hom_i1, p_hom_i2, C2_R_C2C1, C2_t_C2C1, K )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here


%   point_tracks = cell(size(p_hom_i1,2)); todo: Preallocate
    
    view_ids = [1 2];
    
    for i=1:size(p_hom_i1,2)
        point_tracks(i) = pointTrack(view_ids,[p_hom_i1(1:2, i)'; p_hom_i2(1:2, i)' ]);
    end
    
    xyzPoints = P_hom_init(1:3,:)';
    
    cameraPoses = table;    
    cameraPoses.ViewId = [uint32(1);uint32(2)];
    cameraPoses.Orientation = [{eye(3)};{C2_R_C2C1}];
    cameraPoses.Location = [{zeros(1, 3)};{C2_t_C2C1'}];
    
    cameraParams = cameraParameters('IntrinsicMatrix', K);
%     
    [P_hom_init, refinedPoses] = bundleAdjustment(xyzPoints, point_tracks, cameraPoses, cameraParams );
%     
    P_hom_init = P_hom_init';
    
    W_T_WC1 = [cell2mat(refinedPoses.Orientation(1)), cell2mat(refinedPoses.Location(1))';
               zeros(1,3),       1];
         
    W_T_WC2 = [cell2mat(refinedPoses.Orientation(2)), cell2mat(refinedPoses.Location(2))';
               zeros(1,3),       1];
    
end

