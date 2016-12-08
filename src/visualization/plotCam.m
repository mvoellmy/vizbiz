function [ ] = plotCam( T_WC, camsize, camcolor )
% Plots a cam in 3D.
% 
% Input:
%  - T_WC(4x4) : Translation Matrix from world to cam
%  - camsize(int) : Size of the cam default 6
%  - camcolor(string) : Color of the cam
%
% More options and overload can be added if neccessary

plotCamera('Location', T_WC(1:3,4),'Orientation', T_WC(1:3,1:3), 'Size', camsize,'Color', camcolor );

end

