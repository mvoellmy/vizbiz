function [img] = getFrame(params, idx)
% Returns current image.
% 
% Input:
%  - params(struct) : parameter struct
%  - idx(1x1) : dataset frame index
%
% Output:
%  - img(size) : current image

if params.ds == 0
    img = imread([params.kitti_path '/00/image_0/' sprintf('%06d.png',idx)]);
elseif params.ds == 1
    images = dir([params.malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    img = rgb2gray(imread([params.malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(idx).name]));
elseif params.ds == 2
    img = im2uint8(rgb2gray(imread([params.parking_path ...
        sprintf('/images/img_%05d.png',idx)])));
else
    assert(false);
end

end
