function M_CW = estimatePoseDLT(Cj_p_uv, Ci_P, K)
% Estimates the pose of a camera using a set of 2D (rotated frame) -3D (original frame) correspondences and a
% given camera matrix (for rotated frame)
%
% p: [nx2] vector containing the undistorted coordinates of the 2D points [u v] most probably
% P: [nx3] vector containing the 3D point positions in frame of camera i
% K: [3x3] camera matrix
%
% M_CW: [3x4] projection matrix under the form M=[R|t] where R is a rotation
%    matrix. M encodes the transformation that maps points from the world
%    frame to the camera frame

% Convert 2D points to normalized coordinates
p_normalized = (K \ [Cj_p_uv ones(length(Cj_p_uv),1)]')';

% Build the measurement matrix Q
num_corners = length(p_normalized);
Q = zeros(2*num_corners, 12);

for i=1:num_corners
    u = p_normalized(i,1);
    v = p_normalized(i,2);
    
    Q(2*i-1,1:3) = Ci_P(i,:);
    Q(2*i-1,4) = 1;
    Q(2*i-1,9:12) = -u * [Ci_P(i,:) 1];
    
    Q(2*i,5:7) = Ci_P(i,:);
    Q(2*i,8) = 1;
    Q(2*i,9:12) = -v * [Ci_P(i,:) 1];
end

% Solve for Q.M = 0 subject to the constraint ||M||=1
[~,~,V] = svd(Q);
M_CW = V(:,end);

M_CW = reshape(M_CW, 4, 3)';

%% Extract [R|t] with the correct scale from M ~ [R|t]

if det(M_CW(:,1:3)) < 0
    M_CW = -M_CW;
end

R = M_CW(:,1:3);

% Find the closest orthogonal matrix to R
% https://en.wikipedia.org/wiki/Orthogonal_Procrustes_problem
[U,~,V] = svd(R);
R_tilde = U*V';

% Normalization scheme using the Frobenius norm:
% recover the unknown scale using the fact that R_corr is a true rotation matrix
alpha = norm(R_tilde, 'fro')/norm(R, 'fro');

% Build M with the corrected rotation and scale
M_CW = [R_tilde alpha * M_CW(:,4)];


end

