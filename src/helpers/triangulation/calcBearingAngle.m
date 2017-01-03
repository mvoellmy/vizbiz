function [bearing_angle_deg] = calcBearingAngle(kp_img1, kp_img2, K)
% todo description
%
% Input: todo
%  - kp_img1(2xN) : [v u]
%  - kp_img2(2xN) : [v u]
%
% Output:
%  - bearing_angle_deg(1xN) : in-between angles betwain kp pairs

vector_first = [kp_img1; repmat(K(1,1), [1, size(kp_img1,2)])];
vector_j = [kp_img2; repmat(K(1,1), [1, size(kp_img2,2)])];

bearing_angle_deg = atan2d(twoNormMatrix(cross(vector_j, vector_first)), dot(vector_j, vector_first));

end
