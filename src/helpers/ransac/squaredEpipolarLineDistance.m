function [squ_error_dist] = squaredEpipolarLineDistance(p_hom_norm_i1, p_hom_norm_i2, F)
% todo
% 
% Input:
%  - p_hom_norm_i1(3xN) : normalized homogenous 2D points of image 1
%  - p_hom_norm_i2(3xN) : normalized homogenous 2D points of image 2
%  - F(3x3) : fundamental matrix
%
% Output:
%  - error_dist(1xN) : euclidean error distances

line_1 = F'*p_hom_norm_i2;
line_2 = F*p_hom_norm_i1;

% squared euclidean distance in pixels
squ_error_dist = (p_hom_norm_i1(1,:)-line_1(1,:)).^2+(p_hom_norm_i1(2,:)-line_1(2,:)).^2 + ...
                 (p_hom_norm_i2(1,:)-line_2(1,:)).^2+(p_hom_norm_i2(2,:)-line_2(2,:)).^2;

end
