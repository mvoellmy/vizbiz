function [error_dist] = geomError2EpipolarLine(p_hom_i1, p_hom_i2, F)
% todo
% 
% Input:
%  - p_hom_i1(3xN) : homogenous 2D points of image 1
%  - p_hom_i2(3xN) : homogenous 2D points of image 2
%  - F(3x3) : fundamental matrix
%
% Output:
%  - error_dist(1xN) : euclidean error distances
               
N = size(p_hom_i1,2);

epi_lines_1 = F.'*p_hom_i2; % todo: check transpose
epi_lines_2 = F*p_hom_i1;

for i=1:N % remove for-loop
    error_dist(i) = 0.5* ( abs(epi_lines_1(:,i)'*p_hom_i1(:,i)) / sqrt(epi_lines_1(1,i).^2 + epi_lines_1(2,i).^2) + ...
                           abs(epi_lines_2(:,i)'*p_hom_i2(:,i)) / sqrt(epi_lines_2(1,i).^2 + epi_lines_2(2,i).^2) );
end

end
