% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 1851.144924807365700 ; 1848.413142532056600 ];

%-- Principal point:
cc = [ 560.115564563555610 ; 924.109017380704360 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.186147989540302 ; -0.839087534450825 ; -0.009148011848673 ; 0.001571395951427 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 17.830012126005720 ; 15.306007655245766 ];

%-- Principal point uncertainty:
cc_error = [ 13.581940237513543 ; 12.960896057000575 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.025827601312203 ; 0.148583414170534 ; 0.003230071171447 ; 0.003636489699470 ; 0.000000000000000 ];

%-- Image size:
nx = 1080;
ny = 1920;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 29;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -1.898850e+00 ; -2.027602e+00 ; 9.425711e-02 ];
Tc_1  = [ -8.960422e+01 ; -1.275033e+02 ; 8.279738e+02 ];
omc_error_1 = [ 6.433729e-03 ; 6.570636e-03 ; 1.357421e-02 ];
Tc_error_1  = [ 6.044773e+00 ; 5.827631e+00 ; 7.409470e+00 ];

%-- Image #2:
omc_2 = [ -1.906111e+00 ; -2.065911e+00 ; 5.162861e-03 ];
Tc_2  = [ -1.356347e+02 ; -1.224666e+02 ; 7.038789e+02 ];
omc_error_2 = [ 6.058336e-03 ; 6.360629e-03 ; 1.329176e-02 ];
Tc_error_2  = [ 5.148569e+00 ; 4.988725e+00 ; 6.565757e+00 ];

%-- Image #3:
omc_3 = [ NaN ; NaN ; NaN ];
Tc_3  = [ NaN ; NaN ; NaN ];
omc_error_3 = [ NaN ; NaN ; NaN ];
Tc_error_3  = [ NaN ; NaN ; NaN ];

%-- Image #4:
omc_4 = [ -1.632165e+00 ; -1.913440e+00 ; 3.432874e-01 ];
Tc_4  = [ -1.275219e+02 ; -1.372180e+02 ; 7.777008e+02 ];
omc_error_4 = [ 6.262968e-03 ; 5.958678e-03 ; 9.644535e-03 ];
Tc_error_4  = [ 5.658925e+00 ; 5.472066e+00 ; 6.281094e+00 ];

%-- Image #5:
omc_5 = [ -1.436226e+00 ; -2.231526e+00 ; 1.027463e-01 ];
Tc_5  = [ -1.485483e+02 ; -2.749605e+01 ; 8.784137e+02 ];
omc_error_5 = [ 5.606585e-03 ; 8.111542e-03 ; 1.201447e-02 ];
Tc_error_5  = [ 6.390109e+00 ; 6.205456e+00 ; 7.222542e+00 ];

%-- Image #6:
omc_6 = [ -1.639544e+00 ; -2.106632e+00 ; -2.356680e-01 ];
Tc_6  = [ -4.294219e+01 ; 9.927082e+00 ; 7.012942e+02 ];
omc_error_6 = [ 4.582110e-03 ; 8.120193e-03 ; 1.308707e-02 ];
Tc_error_6  = [ 5.092391e+00 ; 4.880301e+00 ; 6.438322e+00 ];

%-- Image #7:
omc_7 = [ -2.168096e+00 ; -2.044658e+00 ; -8.400273e-01 ];
Tc_7  = [ -6.242162e+01 ; -6.936232e+01 ; 6.438349e+02 ];
omc_error_7 = [ 4.647062e-03 ; 7.461116e-03 ; 1.301765e-02 ];
Tc_error_7  = [ 4.696590e+00 ; 4.541792e+00 ; 6.187560e+00 ];

%-- Image #8:
omc_8 = [ NaN ; NaN ; NaN ];
Tc_8  = [ NaN ; NaN ; NaN ];
omc_error_8 = [ NaN ; NaN ; NaN ];
Tc_error_8  = [ NaN ; NaN ; NaN ];

%-- Image #9:
omc_9 = [ 1.487329e+00 ; 2.325556e+00 ; 6.972941e-01 ];
Tc_9  = [ 2.955234e+01 ; -1.536204e+02 ; 6.089250e+02 ];
omc_error_9 = [ 6.508122e-03 ; 7.328395e-03 ; 1.075395e-02 ];
Tc_error_9  = [ 4.500627e+00 ; 4.293637e+00 ; 5.504215e+00 ];

%-- Image #10:
omc_10 = [ 1.064396e+00 ; 2.382692e+00 ; 8.051422e-01 ];
Tc_10  = [ 9.340408e+01 ; -2.431349e+02 ; 6.342293e+02 ];
omc_error_10 = [ 5.291275e-03 ; 8.254328e-03 ; 1.003644e-02 ];
Tc_error_10  = [ 4.825124e+00 ; 4.491168e+00 ; 6.142980e+00 ];

%-- Image #11:
omc_11 = [ NaN ; NaN ; NaN ];
Tc_11  = [ NaN ; NaN ; NaN ];
omc_error_11 = [ NaN ; NaN ; NaN ];
Tc_error_11  = [ NaN ; NaN ; NaN ];

%-- Image #12:
omc_12 = [ NaN ; NaN ; NaN ];
Tc_12  = [ NaN ; NaN ; NaN ];
omc_error_12 = [ NaN ; NaN ; NaN ];
Tc_error_12  = [ NaN ; NaN ; NaN ];

%-- Image #13:
omc_13 = [ NaN ; NaN ; NaN ];
Tc_13  = [ NaN ; NaN ; NaN ];
omc_error_13 = [ NaN ; NaN ; NaN ];
Tc_error_13  = [ NaN ; NaN ; NaN ];

%-- Image #14:
omc_14 = [ NaN ; NaN ; NaN ];
Tc_14  = [ NaN ; NaN ; NaN ];
omc_error_14 = [ NaN ; NaN ; NaN ];
Tc_error_14  = [ NaN ; NaN ; NaN ];

%-- Image #15:
omc_15 = [ -1.931388e+00 ; -1.936948e+00 ; -4.337378e-01 ];
Tc_15  = [ -1.602051e+02 ; 3.339924e+00 ; 7.611553e+02 ];
omc_error_15 = [ 5.993883e-03 ; 7.556525e-03 ; 1.195574e-02 ];
Tc_error_15  = [ 5.566448e+00 ; 5.379464e+00 ; 7.608790e+00 ];

%-- Image #16:
omc_16 = [ -1.955461e+00 ; -1.956611e+00 ; -7.305483e-01 ];
Tc_16  = [ -7.897473e+01 ; 1.632304e+02 ; 8.569241e+02 ];
omc_error_16 = [ 6.257026e-03 ; 9.248612e-03 ; 1.325243e-02 ];
Tc_error_16  = [ 6.319932e+00 ; 5.978944e+00 ; 8.163927e+00 ];

%-- Image #17:
omc_17 = [ NaN ; NaN ; NaN ];
Tc_17  = [ NaN ; NaN ; NaN ];
omc_error_17 = [ NaN ; NaN ; NaN ];
Tc_error_17  = [ NaN ; NaN ; NaN ];

%-- Image #18:
omc_18 = [ NaN ; NaN ; NaN ];
Tc_18  = [ NaN ; NaN ; NaN ];
omc_error_18 = [ NaN ; NaN ; NaN ];
Tc_error_18  = [ NaN ; NaN ; NaN ];

%-- Image #19:
omc_19 = [ -1.881902e+00 ; -1.898357e+00 ; 4.275628e-02 ];
Tc_19  = [ -7.367821e+01 ; -1.244149e+02 ; 8.076423e+02 ];
omc_error_19 = [ 6.030413e-03 ; 6.236623e-03 ; 1.201555e-02 ];
Tc_error_19  = [ 5.905007e+00 ; 5.691929e+00 ; 7.225012e+00 ];

%-- Image #20:
omc_20 = [ NaN ; NaN ; NaN ];
Tc_20  = [ NaN ; NaN ; NaN ];
omc_error_20 = [ NaN ; NaN ; NaN ];
Tc_error_20  = [ NaN ; NaN ; NaN ];

%-- Image #21:
omc_21 = [ NaN ; NaN ; NaN ];
Tc_21  = [ NaN ; NaN ; NaN ];
omc_error_21 = [ NaN ; NaN ; NaN ];
Tc_error_21  = [ NaN ; NaN ; NaN ];

%-- Image #22:
omc_22 = [ NaN ; NaN ; NaN ];
Tc_22  = [ NaN ; NaN ; NaN ];
omc_error_22 = [ NaN ; NaN ; NaN ];
Tc_error_22  = [ NaN ; NaN ; NaN ];

%-- Image #23:
omc_23 = [ NaN ; NaN ; NaN ];
Tc_23  = [ NaN ; NaN ; NaN ];
omc_error_23 = [ NaN ; NaN ; NaN ];
Tc_error_23  = [ NaN ; NaN ; NaN ];

%-- Image #24:
omc_24 = [ NaN ; NaN ; NaN ];
Tc_24  = [ NaN ; NaN ; NaN ];
omc_error_24 = [ NaN ; NaN ; NaN ];
Tc_error_24  = [ NaN ; NaN ; NaN ];

%-- Image #25:
omc_25 = [ -2.229676e+00 ; -1.607965e+00 ; -9.591562e-01 ];
Tc_25  = [ -5.024577e+01 ; 5.245706e+01 ; 7.077240e+02 ];
omc_error_25 = [ 5.482425e-03 ; 7.263465e-03 ; 1.218600e-02 ];
Tc_error_25  = [ 5.188269e+00 ; 4.935581e+00 ; 6.744504e+00 ];

%-- Image #26:
omc_26 = [ NaN ; NaN ; NaN ];
Tc_26  = [ NaN ; NaN ; NaN ];
omc_error_26 = [ NaN ; NaN ; NaN ];
Tc_error_26  = [ NaN ; NaN ; NaN ];

%-- Image #27:
omc_27 = [ 1.466213e+00 ; 1.828262e+00 ; 4.676375e-01 ];
Tc_27  = [ -4.840926e+01 ; -1.872494e+02 ; 7.168142e+02 ];
omc_error_27 = [ 5.816310e-03 ; 6.319741e-03 ; 9.719332e-03 ];
Tc_error_27  = [ 5.276194e+00 ; 5.013941e+00 ; 6.035323e+00 ];

%-- Image #28:
omc_28 = [ NaN ; NaN ; NaN ];
Tc_28  = [ NaN ; NaN ; NaN ];
omc_error_28 = [ NaN ; NaN ; NaN ];
Tc_error_28  = [ NaN ; NaN ; NaN ];

%-- Image #29:
omc_29 = [ -2.202227e+00 ; -1.751529e+00 ; 8.368419e-01 ];
Tc_29  = [ -1.727340e+02 ; -1.939371e+02 ; 9.032802e+02 ];
omc_error_29 = [ 8.176106e-03 ; 3.777813e-03 ; 1.130543e-02 ];
Tc_error_29  = [ 6.656759e+00 ; 6.380099e+00 ; 7.075975e+00 ];

