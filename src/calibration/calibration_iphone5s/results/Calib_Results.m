% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 2774.775440591392500 ; 2784.563736081753200 ];

%-- Principal point:
cc = [ 1240.187164121787600 ; 1607.530245683252100 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.056878797350909 ; -0.048096292485037 ; -0.003448969063450 ; 0.000755342633803 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 9.327982331895866 ; 9.090216669425569 ];

%-- Principal point uncertainty:
cc_error = [ 13.021277906197664 ; 19.489725486849967 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.013844700661017 ; 0.042496150304482 ; 0.002727687333200 ; 0.001704536093118 ; 0.000000000000000 ];

%-- Image size:
nx = 2448;
ny = 3264;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 11;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -2.063642e+00 ; -2.122616e+00 ; -8.915809e-02 ];
Tc_1  = [ -1.235065e+02 ; -1.123286e+02 ; 8.650240e+02 ];
omc_error_1 = [ 6.172918e-03 ; 5.992288e-03 ; 1.218849e-02 ];
Tc_error_1  = [ 4.056580e+00 ; 6.119422e+00 ; 3.394949e+00 ];

%-- Image #2:
omc_2 = [ -1.531490e+00 ; -1.893639e+00 ; 5.315558e-01 ];
Tc_2  = [ -6.896483e+01 ; -7.757711e+01 ; 7.438699e+02 ];
omc_error_2 = [ 5.283376e-03 ; 4.028022e-03 ; 7.985647e-03 ];
Tc_error_2  = [ 3.464244e+00 ; 5.213553e+00 ; 2.335084e+00 ];

%-- Image #3:
omc_3 = [ -1.387721e+00 ; -1.898692e+00 ; 3.274689e-01 ];
Tc_3  = [ -2.558250e+01 ; -3.147051e+01 ; 8.136532e+02 ];
omc_error_3 = [ 4.944202e-03 ; 4.479359e-03 ; 7.942318e-03 ];
Tc_error_3  = [ 3.796757e+00 ; 5.695526e+00 ; 2.670793e+00 ];

%-- Image #4:
omc_4 = [ -2.173401e+00 ; -2.171377e+00 ; -7.597584e-02 ];
Tc_4  = [ -1.075481e+02 ; -7.092398e+01 ; 3.849514e+02 ];
omc_error_4 = [ 3.670841e-03 ; 4.115724e-03 ; 8.498637e-03 ];
Tc_error_4  = [ 1.806345e+00 ; 2.749029e+00 ; 1.492126e+00 ];

%-- Image #5:
omc_5 = [ 1.826944e+00 ; 1.334241e+00 ; 1.063268e+00 ];
Tc_5  = [ -5.006696e+01 ; -8.390599e+01 ; 3.921385e+02 ];
omc_error_5 = [ 5.959788e-03 ; 3.720948e-03 ; 7.449018e-03 ];
Tc_error_5  = [ 1.854045e+00 ; 2.745586e+00 ; 1.753738e+00 ];

%-- Image #6:
omc_6 = [ 1.380448e+00 ; 1.512351e+00 ; 1.147073e-01 ];
Tc_6  = [ -9.832637e+01 ; -9.008296e+01 ; 5.507662e+02 ];
omc_error_6 = [ 5.634655e-03 ; 3.847263e-03 ; 6.845932e-03 ];
Tc_error_6  = [ 2.566902e+00 ; 3.880691e+00 ; 1.783033e+00 ];

%-- Image #7:
omc_7 = [ 2.054110e+00 ; 1.690353e+00 ; 7.545854e-01 ];
Tc_7  = [ -6.975965e+01 ; -1.518568e+02 ; 1.218916e+03 ];
omc_error_7 = [ 6.193872e-03 ; 4.118721e-03 ; 1.045380e-02 ];
Tc_error_7  = [ 5.741222e+00 ; 8.534890e+00 ; 5.012038e+00 ];

%-- Image #8:
omc_8 = [ -2.292279e+00 ; -1.387073e+00 ; -1.392937e+00 ];
Tc_8  = [ -7.573888e+01 ; -2.105200e+01 ; 3.833171e+02 ];
omc_error_8 = [ 4.452949e-03 ; 7.200832e-03 ; 7.335471e-03 ];
Tc_error_8  = [ 1.805226e+00 ; 2.699604e+00 ; 1.642102e+00 ];

%-- Image #9:
omc_9 = [ -1.947789e+00 ; -2.111194e+00 ; 2.161261e-01 ];
Tc_9  = [ -1.115698e+02 ; -1.664875e+02 ; 1.500441e+03 ];
omc_error_9 = [ 8.905199e-03 ; 8.024897e-03 ; 1.567808e-02 ];
Tc_error_9  = [ 7.043801e+00 ; 1.052036e+01 ; 5.734331e+00 ];

%-- Image #10:
omc_10 = [ -1.832881e+00 ; -1.476467e+00 ; -3.206395e-01 ];
Tc_10  = [ -7.194547e+01 ; -5.169384e+01 ; 7.061057e+02 ];
omc_error_10 = [ 5.187823e-03 ; 4.422340e-03 ; 7.060083e-03 ];
Tc_error_10  = [ 3.310723e+00 ; 4.951209e+00 ; 2.460303e+00 ];

%-- Image #11:
omc_11 = [ 1.619212e+00 ; 2.125930e+00 ; 3.585192e-01 ];
Tc_11  = [ -5.442667e+01 ; -1.488013e+02 ; 8.894427e+02 ];
omc_error_11 = [ 4.798270e-03 ; 4.404563e-03 ; 1.055869e-02 ];
Tc_error_11  = [ 4.180758e+00 ; 6.235905e+00 ; 3.364339e+00 ];

