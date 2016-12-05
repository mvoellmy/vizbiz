function [error_dist] = algError2EpipolarLine(p_hom_i1, p_hom_i2, F)
% todo
% 
% Input:
%  - p_hom_i1(3xN) : homogenous 2D points of image 1
%  - p_hom_i2(3xN) : homogenous 2D points of image 2
%  - F(3x3) : fundamental matrix
%
% Output:
%  - error_dist(1xN) : algebraic error distances (x2(i).' * F * x1(i) = 0)

N = size(p_hom_i1,2);
for i=1:N
    error_dist(i) = abs(p_hom_i2(:,i)'*(F*p_hom_i1(:,i))); % todo remove for loop
end

end
