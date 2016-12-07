function extractImagesFromVideo(video_path,options)

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
        thisframe = rot90(read(vidobj,f),-1);

        figure(fig);
        imagesc(thisframe);
        axis equal;
        axis off;

        if options.save_images
            i = n_extracted + 1;
            thisfile = sprintf('./calib_images/camera_calib%04d.jpg',i); % todo: png good format?
            imwrite(thisframe,thisfile);
            n_extracted = n_extracted + 1;
        end
    end
    
    waitbar(f/frames);
    pause(0.01);
end

close(h);
close all;

% display summary
fprintf(['Calibration sequence %f seconds\n',...
         '%i frames contained\n',...
         '%i calibration frames extracted\n\n'],...
         vidobj.Duration,frames,n_extracted);

end
