clear;
close all;
clc;

% set desired options
options.extract_every_x = 3; % for all images set: 1
options.convert_to_grayscale = true;
options.rotate_images = 0; % rotates images k*90°, 1: Counterclock wise, 0: No Rotation, -1: Clockwise
options.save_images = true;

%% 
[~, path_name] = uigetfile('*.*','Select the first image...');
convertImages2CalibImages(path_name, options);

%%
[video_name, path_name] = uigetfile('*.*','Select the video source file...');
extractImagesFromVideo([path_name, video_name], options);
