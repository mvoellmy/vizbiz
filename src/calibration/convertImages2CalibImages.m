function convertImages2CalibImages(images_path,options)
% Converts images from a user-specified directory into calibration images  
% with predefined naming convention (as required by calibration tool).
%
% Input:
%  - images_path(path) : image directory path
%  - options(struct) : options structure

fig = figure('name','Extracted images');

if options.save_images
    folder_name = 'calib_images2';
    if isequal(exist(folder_name,'dir'),7)
        rmdir(folder_name,'s');
    end
    mkdir(folder_name);
    
    n_extracted = 0;
end

h = waitbar(0,'Extracting images...','Units','normalized','OuterPosition',[0.35 0.2 0.3 0.1]);

imagefiles = dir([images_path,'/*.jpg']);
n_images = length(imagefiles);
for i=1:n_images
   
    if ~mod(i,options.extract_every_x)
        thisframe = rot90(imread([images_path,imagefiles(i).name]),options.rotate_images);
        if options.convert_to_grayscale
            thisframe = rgb2gray(thisframe);
        end

        figure(fig);
        imagesc(thisframe);
        axis equal;
        axis off;
        
        if options.save_images
            i = n_extracted + 1;
            thisfile = sprintf(['./',folder_name,'/camera_calib%04d.jpg'],i); % todo: jpg good format?
            imwrite(thisframe,thisfile);
            title(['Frame #',num2str(i)]);
            
            n_extracted = n_extracted + 1;
        end
    end
    
    waitbar(i/n_images);
    pause(0.01);
end

close(h);
close all;

% display summary
fprintf(['%i frames contained\n',...         
         '%i calibration frames extracted\n\n'],...
         n_images,n_extracted);

end
