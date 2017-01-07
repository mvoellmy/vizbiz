clear;
clc;

p_success = 0.999999;
fract_inliers = 0.3;

s = 3; % p3p

num_iterations = ceil(log(1-p_success)/...
                      log(1-fract_inliers^s))
