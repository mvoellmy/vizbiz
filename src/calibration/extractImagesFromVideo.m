function extractImagesFromVideo(video_path,options)

fig = figure('name','Extracted images');

if options.save_images
    mkdir('calib_images');
end

vidobj = VideoReader(video_path);
frames = vidobj.Numberofframes;

h = waitbar(0,'Extracting images...','Units','normalized','OuterPosition',[0.35 0.2 0.3 0.1]);

for f=1:frames
    thisframe = rot90(read(vidobj,f),-1);

    figure(fig);
    imagesc(thisframe);
    axis equal;
    axis off;
    
    if options.save_images
        thisfile = sprintf('./calib_images/camera_calib%04d.jpg',f); % todo: png good format?
        imwrite(thisframe,thisfile);        
    end
    
    waitbar(f/frames);
    pause(0.01);
end

close(h);
close all;

% display summary
fprintf(['Calibration sequence %f seconds\n',...
         '%i frames were extracted\n'],...
         vidobj.Duration,frames);

end
