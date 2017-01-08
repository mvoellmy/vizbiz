function [error_dist] = geomError2EpipolarLine(p_hom_i1, p_hom_i2, F)
% todo
% 
% Input:
%  - p_hom_i1(3xN) : homogenous 2D points of image 1
%  - p_hom_i2(3xN) : homogenous 2D points of image 2
%  - F(3x3) : fundamental matrix
%
% Output:
%  - error_dist(1xN) : error pixel distances
        
app = 4; % todo: which correct/best??

% first approach: self
% N = size(p_hom_i1,2);
% epi_lines_1 = F' * p_hom_i2;
% epi_lines_2 = F * p_hom_i1;
% for i=1:N % todo: vectorize for-loop!
%     error_dist_1(i) = ( abs(epi_lines_1(:,i)'*p_hom_i1(:,i)) / sqrt(epi_lines_1(1,i)^2 + epi_lines_1(2,i)^2) + ...
%                         abs(epi_lines_2(:,i)'*p_hom_i2(:,i)) / sqrt(epi_lines_2(1,i)^2 + epi_lines_2(2,i)^2) );
% end
% 
% % second approach: First-order geometric error (Sampson distance)
% % ("Multiple View Geometry" Hartley & Zisserman 2000. page 287)
% a1 = F*p_hom_i1; % epipolar lines in image 2
% a2 = F.'*p_hom_i2; % epipolar lines in image 1
% Numer = sum(p_hom_i2.*a1, 1).^2; 
% Denom = a1(1,:).^2 + a1(2,:).^2 + a2(1,:).^2 + a2(2,:).^2;
% dSquared = Numer./Denom;
% error_dist_2 = sqrt(dSquared);
% 
% % third approach: Symmetric epipolar line distance
% % ("Multiple View Geometry" Hartley & Zisserman 2000. page 288)
% a1 = F*p_hom_i1; % epipolar lines in image 2
% a2 = F.'*p_hom_i2; % epipolar lines in image 1
% Numer = sum(p_hom_i2.*a1, 1).^2; 
% Denom = 1 ./ (a1(1,:).^2 + a1(2,:).^2) + 1 ./ (a2(1,:).^2 + a2(2,:).^2);
% dSquared = Numer.*Denom;
% error_dist_3 = sqrt(dSquared);

% fourth approach: exercise
NumPoints = size(p_hom_i1,2);
homog_points = [p_hom_i1, p_hom_i2];
epi_lines = [F.'*p_hom_i2, F*p_hom_i1]; % todo: check .' 
denom = epi_lines(1,:).^2 + epi_lines(2,:).^2;
cost = sqrt( (sum(epi_lines.*homog_points,1).^2)./denom );
error_dist_4 = sum(reshape(cost,NumPoints,2)',1);



if app == 1
    error_dist = error_dist_1;
elseif app == 2
    error_dist = error_dist_2;
elseif app == 3
    error_dist = error_dist_3;
elseif app == 4
    error_dist = error_dist_4;
end

end
