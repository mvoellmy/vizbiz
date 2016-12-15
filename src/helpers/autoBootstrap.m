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

if params.init.show_bootstrap_images
    figure('name','Bootstrapping');
end
% todo: use params.init.show_bootstrap_images

if params.auto_bootstrap
    bootstrap_frame_idx_1 = 1; %bootstrapFrames(params.ds,'first'); % todo

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

    if params.init.show_bootstrap_images
        % show first bootstrap image
        subplot(4,1,1);
        imshow(img0);
        axis equal;
        title('Bootstrap frame 1');
    end

    % search for second bootstrapping image
    bootstrap_pair_found = false;
    frame_idx_candidate = bootstrap_frame_idx_1;

    while ~bootstrap_pair_found    
        frame_idx_candidate = frame_idx_candidate + 1;

        % read in candidate image
        img_candidate = imread([params.kitti_path '/00/image_0/' ...
            sprintf('%06d.png',frame_idx_candidate)]);

        if params.init.show_bootstrap_images
            % show anaglyph
            subplot(4,1,2);
            imshow(stereoAnaglyph(img0,img_candidate));
            axis equal;
            title('Red-cyan composite view of image pair');
        end
        
        % compute disparity
        disparityRange = [-6 10]; % todo: correct values?
        disparityMap = disparity(img0,img_candidate,'BlockSize',...
                                 15,'DisparityRange',disparityRange);

        if params.init.show_bootstrap_images
            subplot(4,1,3);
            imshow(disparityMap,disparityRange);
            axis equal;
            title('Disparity Map');

            subplot(4,1,4);
            imshow(img_candidate);
            axis equal;
            title('Bootstrap frame 2');
        end
        
        % todo: decide wether candidate is suited as bootstrap image
        if frame_idx_candidate == 3
           img1 = img_candidate;
           bootstrap_frame_idx_2 = frame_idx_candidate;
           bootstrap_pair_found = true;
        end
    end

else
    bootstrap_frame_idx_1 = 1;
    bootstrap_frame_idx_2 = bootstrapFrames(params.ds,'second');
    
    if params.ds == 0
        img0 = imread([params.kitti_path '/00/image_0/' ...
            sprintf('%06d.png',bootstrap_frame_idx_1)]);
        img1 = imread([params.kitti_path '/00/image_0/' ...
            sprintf('%06d.png',bootstrap_frame_idx_2)]);
    elseif params.ds == 1
        images = dir([params.malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
        left_images = images(3:2:end);
        img0 = rgb2gray(imread([params.malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(bootstrap_frame_idx_1).name]));
        img1 = rgb2gray(imread([params.malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(bootstrap_frame_idx_2).name]));
    elseif params.ds == 2
        img0 = rgb2gray(imread([params.parking_path ...
            sprintf('/images/img_%05d.png',bootstrap_frame_idx_1)]));
        img1 = rgb2gray(imread([params.parking_path ...
            sprintf('/images/img_%05d.png',bootstrap_frame_idx_2)]));
    else
        assert(false);
    end
    
    subplot(2,1,1);
    imshow(img0);
    axis equal;
    title('Bootstrap frame 1');
    
    subplot(2,1,2);
    imshow(img1);
    axis equal;
    title('Bootstrap frame 2');
end

% display frames chosen
fprintf(['  Bootstrap image 1 index: %i\n',...
         '  Bootstrap image 2 index: %i\n'],...
         bootstrap_frame_idx_1, bootstrap_frame_idx_2);

end
