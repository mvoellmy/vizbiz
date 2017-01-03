function gui_updateImage(img, img_handle)
% todo: add description
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
