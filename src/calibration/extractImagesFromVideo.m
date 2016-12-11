function extractImagesFromVideo(video_path,options)
% Extracts image sequence from video and stores images as calibration
% images with predefined naming convention (as required by calibration tool).
%
% Input:
%  - video_path(path) : video sourc path
%  - options(struct) : options structure

fig = figure('name','Extracted images');

if options.save_images
    folder_name = 'calib_images';
    if isequal(exist(folder_name,'dir'),7)
        rmdir(folder_name,'s');
    end
    mkdir(folder_name);
    
    n_extracted = 0;
end

vidobj = VideoReader(video_path);
frames = vidobj.Numberofframes;

h = waitbar(0,'Extracting images...','Units','normalized','OuterPosition',[0.35 0.2 0.3 0.1]);

for f=1:frames
    
    if ~mod(f,options.extract_every_x)
        thisframe = rot90(read(vidobj,f),options.rotate_images);
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
            title(['Frame #',num2str(f)]);
            
            n_extracted = n_extracted + 1;
        end
    end
    
    waitbar(f/frames);
    pause(0.01);
end

close(h);
close all;

% display summary
fprintf(['Calibration sequence %.2f seconds\n',...
         'frame rate %.2f Hz\n',...
         '%i frames contained\n',...         
         '%i calibration frames extracted\n\n'],...
         vidobj.Duration,vidobj.FrameRate,frames,n_extracted);

end
