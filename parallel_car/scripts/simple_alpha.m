% compute the optimal alpha for 

% symbolic variables of the pose of wx
syms w11 w12 w13
syms w21 w22 w23
syms w31 w32 w33

R_o_to_wx = [w11, w12, w13;
    w21, w22, w23;
    w31, w32, w33];

% rotation matrix
syms Rot_Z(z)

Rot_Z(z) = [cos(z), -sin(z), 0;
    sin(z), cos(z), 0;
    0, 0, 1];

% translation matrix
syms Trans_Z(z)

Trans_Z(z) = [1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, z;
    0, 0, 0, 1];

syms a

Rot_alpha = Rot_Z(a);

% y-axis in wx_link coordinate
j = [0, 1, 0]';

global_j = R_o_to_wx * Rot_alpha * j;

% x-axis in wx_link coordinate
i = [1, 0, 0]';

global_i = R_o_to_wx * Rot_alpha * i;


