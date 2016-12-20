function [ T_F2F1 ] = invTansformationMatrix( T_F1F2 )
% Inverts 4x4 transformation matrix

% Input:    T_F1F2: 4x4 homogenious transformation matrix
% Output:   T_F2F1: 4x4 inverted homogenious transformation matrix

R_F1F2 = T_F1F2(1:3,1:3);
F1_t_F1F2 = T_F1F2(1:3,4);
    
T_F2F1 = [R_F1F2',    -R_F1F2'*F1_t_F1F2;
          zeros(1,3),    1                ];    

end

