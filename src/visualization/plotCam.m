function plotCam(T_WC, camsize, camcolor)
% Plots a cam in 3D.
% 
% Input:
%  - T_WC(4x4xN) : Transformation matrix from C to W
%  - camsize(int) : Size of the cam default 6
%  - camcolor(string) : Color of the cam
%
% Output: none
%
% More options and overload can be added if neccessary

for c=1:size(T_WC,3)
    plotCamera('Location', T_WC(1:3,4,c),'Orientation', T_WC(1:3,1:3,c)', 'Size', camsize,'Color', camcolor );
end

end
