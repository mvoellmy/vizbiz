function test_linearTriangulation()

K = [7.188560000000e+02 0 6.071928000000e+02    % kitti
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];


    
p1 = [K(1,3); K(2,3)];
%p1 = flipud(p1);
p1 = [p1; 1];
p2 = p1;


M1 = K*eye(3,4);
theta = -pi/2;
M2 = K*[cos(theta)    0  sin(theta)  2;
      0             1            0  0;
      -sin(theta)    0   cos(theta)  2];  

P = linearTriangulation(p1,p2,M1,M2)

figure();
plotLandmarks(P(1:3));

end