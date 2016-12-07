function plot3DFrustrum(T_wc, varargin)
% todo
% projectFrustrum projects a wireframe camera frusturm pointing in z
%   given world position and orientation
%   varargin = scale

    if nargin == 2
        scale = varargin{1};
    else
        scale = 1;
    end    
    
    c_dx = 0.1;
    c_dy = 0.06;
    c_dz = 0.2;    
    
    % vertices in camera frame
    c_x = scale *[0  -c_dx    c_dx    c_dx   -c_dx];
    c_y = scale *[0  -c_dy   -c_dy    c_dy    c_dy];
    c_z = scale *[0   c_dz    c_dz    c_dz    c_dz];
    
    % transform to world frame
    w_r_IP = T_wc * [c_x; c_y; c_z; ones(1,5)];
    
    w_x = w_r_IP(1,:);
    w_y = w_r_IP(2,:);
    w_z = w_r_IP(3,:);
    
    pairs = [1 2; 1 3; 1 4; 1 5;
             2 3; 3 4; 4 5; 5 2];
    
    frustrum.n_vertices = 5;
    frustrum.x = [];
    frustrum.y = [];
    frustrum.z = [];
    for p=1:size(pairs,1)
        frustrum.x = [frustrum.x, w_x(pairs(p,1)), w_x(pairs(p,2))];
        frustrum.y = [frustrum.y, w_y(pairs(p,1)), w_y(pairs(p,2))];
        frustrum.z = [frustrum.z, w_z(pairs(p,1)), w_z(pairs(p,2))];
    end
   
    for v=1:2:length(frustrum.x)
        hold on;
        plot3(gca,...
              [frustrum.x(v), frustrum.x(v+1)],...
              [frustrum.y(v), frustrum.y(v+1)],...
              [frustrum.z(v), frustrum.z(v+1)],'color','b');
    end
end
