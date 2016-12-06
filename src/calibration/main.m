clear all;
close all;
clc;

video_path = 'C:\Users\Fabio\Desktop\calib_seq.mov';

options.save_images = true;

extractImagesFromVideo(video_path,options);
