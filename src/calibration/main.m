clear all;
close all;
clc;

[video_name,path_name] = uigetfile('*.*','Select the video source file');

options.save_images = true;
options.extract_every_x = 5; % all: 1

extractImagesFromVideo([path_name,video_name],options);
