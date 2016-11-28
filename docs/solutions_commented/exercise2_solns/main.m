close all;
clear all;

%% Load an (undistorted) image and the detected corners
img_index = 1;

undimg_path = sprintf('../data/images_undistorted/img_%04d.jpg', img_index);
undimg = imread(undimg_path);

K = load('../data/K.txt');

p_W_corners = 0.01 * load('../data/p_W_corners.txt');
num_corners = length(p_W_corners);

% Load the 2D projected points (detected on the undistorted image)
all_pts2d = load('../data/detected_corners.txt');
pts2d = all_pts2d(img_index,:);
pts2d = reshape(pts2d, 2, 12)';


%% Now that we have the 2D <-> 3D correspondences (pts2d+normalized <-> p_W_corners),
%% let's find the camera pose with respect to the world using DLT

M_dlt = estimatePoseDLT(pts2d, p_W_corners, K);

%% Plot the original 2D points and the reprojected ones on the image

p_reproj = reprojectPoints(p_W_corners, M_dlt, K);

figure(1);
imshow(undimg); hold on;
plot(pts2d(:,1), pts2d(:,2), 'o'); hold on;
plot(p_reproj(:,1), p_reproj(:,2), '+');
legend('Original points','Reprojected points');
