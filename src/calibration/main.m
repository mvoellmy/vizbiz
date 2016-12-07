clear all;
close all;
clc;

[video_name,path_name] = uigetfile('*.*','Select the video source file...');

options.save_images = true;
options.extract_every_x = 10; % for all images set: 1
options.rotate_images = 0; % rotates images k*90°, 1: Counterclock wise, 0: No Rotation, -1: Clockwise

extractImagesFromVideo([path_name,video_name],options);
