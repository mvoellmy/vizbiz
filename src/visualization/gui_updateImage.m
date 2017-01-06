function gui_updateImage(img, img_handle)
% Updates current frame.
% 
% Input:
%  - img(size) : current image
%  - img_handle(handle) : gui image handle
%
% Output: none

axes(img_handle);
imshow(img);

axis equal;
axis off;
