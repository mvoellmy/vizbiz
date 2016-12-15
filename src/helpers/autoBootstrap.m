function [img0, img1, bootstrap_frame_idx_1, bootstrap_frame_idx_2] = autoBootstrap(params)
% Returns bootstrap image pair and corresponding indices.
% 
% Input:
%  - params(struct) : parameter struct
%
% Output:
%  - img0(size) : first bootstrap image
%  - img1(size) : second bootstrap image
%  - bootstrap_frame_idx_1(1x1) : dataset image index of img0
%  - bootstrap_frame_idx_2(1x1) : dataset image index of img1

figure('name','Bootstrapping');
% todo: use params.init.show_bootstrap_images

if params.auto_bootstrap
    bootstrap_frame_idx_1 = bootstrapFrames(params.ds,'first'); % todo

    if params.ds == 0
        img0 = imread([params.kitti_path '/00/image_0/' ...
            sprintf('%06d.png',bootstrap_frame_idx_1)]);
    elseif params.ds == 1
        images = dir([params.malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
        left_images = images(3:2:end);
        img0 = rgb2gray(imread([params.malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(bootstrap_frame_idx_1).name]));
    elseif params.ds == 2
        img0 = rgb2gray(imread([params.parking_path ...
            sprintf('/images/img_%05d.png',bootstrap_frame_idx_1)]));
    else
        assert(false);
    end

    % show first bootstrap image
    subplot(4,1,1);
    imshow(img0);
    axis equal;
    title('Bootstrap frame 1');

    % search for second bootstrapping image
    bootstrap_pair_found = false;
    frame_idx_candidate = bootstrap_frame_idx_1;

    while ~bootstrap_pair_found    
        frame_idx_candidate = frame_idx_candidate + 1;

        % read in candidate image
        img_candidate = imread([params.kitti_path '/00/image_0/' ...
            sprintf('%06d.png',frame_idx_candidate)]);

        % show anaglyph
        subplot(4,1,2);
        imshow(stereoAnaglyph(img0,img_candidate));
        axis equal;
        title('Red-cyan composite view of image pair');

        % compute disparity
        disparityRange = [-6 10]; % todo
        disparityMap = disparity(img0,img_candidate,'BlockSize',...
                                 15,'DisparityRange',disparityRange);

        subplot(4,1,3);
        imshow(disparityMap,disparityRange);
        axis equal;
        title('Disparity Map');
        colormap jet;
        colorbar;

        subplot(4,1,4);
        imshow(img_candidate);
        axis equal;
        title('Bootstrap frame 2');

        % decide wether candidate is suited as bootstrap image
        if frame_idx_candidate == 3
           % todo
           img1 = img_candidate;
           bootstrap_frame_idx_2 = frame_idx_candidate;
           bootstrap_pair_found = true;
        end
    end

else
    if params.ds == 0
        img0 = imread([params.kitti_path '/00/image_0/' ...
            sprintf('%06d.png',bootstrapFrames(params.ds,'first'))]);
        img1 = imread([params.kitti_path '/00/image_0/' ...
            sprintf('%06d.png',bootstrapFrames(params.ds,'second'))]);
    elseif params.ds == 1
        images = dir([params.malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
        left_images = images(3:2:end);
        img0 = rgb2gray(imread([params.malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(bootstrapFrames(params.ds,'first')).name]));
        img1 = rgb2gray(imread([params.malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(bootstrapFrames(params.ds,'second')).name]));
    elseif params.ds == 2
        img0 = rgb2gray(imread([params.parking_path ...
            sprintf('/images/img_%05d.png',bootstrapFrames(params.ds,'first'))]));
        img1 = rgb2gray(imread([params.parking_path ...
            sprintf('/images/img_%05d.png',bootstrapFrames(params.ds,'second'))]));
    else
        assert(false);
    end
end

% todo: display frames chosen

end
