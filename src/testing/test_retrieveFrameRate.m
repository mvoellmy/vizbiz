clear;
clc;

% first 10 time intervals of kitti dataset
times = [
            0.000000e+00
            1.037359e-01
            2.073381e-01
            3.110752e-01
            4.146917e-01
            5.184302e-01
            6.220448e-01
            7.257977e-01
            8.294199e-01
            9.331467e-01
            1.036910e+00 ];

delta_times_sec = times(2:end) - times(1:end-1);
frame_rate_hz = mean(1./delta_times_sec);

fprintf('Kitti frame rate [hz]: %.2f\n', frame_rate_hz);
