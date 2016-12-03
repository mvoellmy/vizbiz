function P_hom = linearTriangulation(p1,p2,M1,M2)
% Triangulates 2D points into 3D space.
%
% Input:
%  - p1(3xN) : homogeneous coordinates of points in image 1
%  - p2(3xN) : homogeneous coordinates of points in image 2
%  - M1(3x4) : projection matrix corresponding to first image
%  - M2(3x4) : projection matrix corresponding to second image
%
% Output:
%  - P_hom(4xN) : homogeneous coordinates of 3-D points

% Sanity checks
[dim,NumPoints] = size(p1);
[dim2,npoints2] = size(p2);
assert(dim==dim2,'Size mismatch of input points');
assert(NumPoints==npoints2,'Size mismatch of input points');
assert(dim==3,'Arguments x1, x2 should be 3xN matrices (homogeneous coords)');

[rows,cols] = size(M1);
assert(rows==3 && cols==4,'Projection matrices should be of size 3x4');
[rows,cols] = size(M2);
assert(rows==3 && cols==4,'Projection matrices should be of size 3x4');

P_hom = zeros(4,NumPoints);

% Linear algorithm
for j=1:NumPoints
    % Built matrix of linear homogeneous system of equations
    A1 = cross2Matrix(p1(:,j))*M1;
    A2 = cross2Matrix(p2(:,j))*M2;
    A = [A1; A2];
    
    % Solve the linear homogeneous system of equations
    [~,~,v] = svd(A,0);
    P_hom(:,j) = v(:,4);
end

P_hom = P_hom./repmat(P_hom(4,:),4,1); % Dehomogeneize (P is expressed in homogeneous coordinates)

end
