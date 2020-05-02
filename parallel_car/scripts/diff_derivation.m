% Construct symbolic functions to conduct differentiation

% symbolic function will be used to construct
syms Rot_X(x) Rot_Y(y) Rot_Z(z)
syms Trans_X(x) Trans_Y(y) Trans_Z(z)

Rot_X(x) = [1, 0, 0, 0;
    0, cos(x), -sin(x), 0;
    0, sin(x), cos(x), 0;
    0, 0, 0, 1];

Rot_Y(y) = [cos(y), 0, sin(y), 0;
    0, 1, 0, 0;
    -sin(y), 0, cos(y), 0;
    0, 0, 0, 1];

Rot_Z(z) = [cos(z), -sin(z), 0, 0;
    sin(z), cos(z), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

Trans_X(x) = [1, 0, 0, x;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

Trans_Y(y) = [1, 0, 0, 0;
    0, 1, 0, y;
    0, 0, 1, 0;
    0, 0, 0, 1];

Trans_Z(z) = [1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, z;
    0, 0, 0, 1];

% symbolic variables of down_link
syms d_a d_r d_h
% d_a: down_angle
% d_r: down_radius
% d_h: down_height

T_down_to_down_num = Rot_Z(d_a)*Trans_X(d_r)*Trans_Z(d_h);

% get inverse of T_down_to_down_num
inv_T_down_to_down_num = [cos(d_a), sin(d_a), 0, -d_r;
    -sin(d_a), cos(d_a), 0, 0;
    0, 0, 1, -d_h;
    0, 0, 0, 1];

% symbolic variables of up_link
syms u_a u_r u_h
% u_a: up_angle
% u_r: up_radius
% u_h: up_height

T_up_to_up_num = Rot_Z(u_a)*Trans_X(u_r)*Trans_Z(u_h);

% get inverse of T_up_to_up_num
inv_T_up_to_up_num = [cos(u_a), sin(u_a), 0, -u_r;
    -sin(u_a), cos(u_a), 0, 0;
    0, 0, 1, -u_h;
    0, 0, 0, 1];

% symbolic variables of the pose of the down_link
syms d_x d_y d_t C_H
% d_x: down_x
% d_y: down_y
% d_t: down_theta
% C_H: CAR_HEIGHT

T_o_to_down = Trans_X(d_x)*Trans_Y(d_y)*Trans_Z(C_H)*Rot_Z(d_t);

% get the inverse of T_o_to_down
inv_T_o_to_down = [cos(d_t), sin(d_t), 0, -(d_x*cos(d_t)+d_y*sin(d_t));
    -sin(d_t), cos(d_t), 0, -(d_y*cos(d_t)-d_x*sin(d_t));
    0, 0, 1, -C_H;
    0, 0, 0, 1];

% symbolic variables of the pose of wx
syms w11 w12 w13 w_x
syms w21 w22 w23 w_y
syms w31 w32 w33 w_z

T_o_to_wx = [w11, w12, w13, w_x;
    w21, w22, w23, w_y;
    w31, w32, w33, w_z;
    0, 0, 0, 1];

% symbolic variables of the pose of wx with respect to up_link
syms w_a a_l
% w_a: wx_alpha
% a_l: addon_length

T_up_to_wx = Trans_Z(a_l) * Rot_Y(pi/4) * Trans_Z(a_l) * Rot_Z(w_a);

% get the inverse of T_up_to_wx
inv_T_up_to_wx = [cos(w_a), sin(w_a), 0, 0;
    -sin(w_a), cos(w_a), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

% start the computation of T_down_num_to_up_num

T_down_num_to_up_num = inv_T_down_to_down_num * inv_T_o_to_down * T_o_to_wx * inv_T_up_to_wx * T_up_to_up_num;

% pole
length_x = T_down_num_to_up_num(1,4);
length_y = T_down_num_to_up_num(2,4);
length_z = T_down_num_to_up_num(3,4);

% independent variables
% d_x, d_y, d_t, w_a

length_x_p_d_x = diff(length_x, d_x);
length_x_p_d_y = diff(length_x, d_y);
length_x_p_d_t = diff(length_x, d_t);
length_x_p_w_a = diff(length_x, w_a);

length_y_p_d_x = diff(length_y, d_x);
length_y_p_d_y = diff(length_y, d_y);
length_y_p_d_t = diff(length_y, d_t);
length_y_p_w_a = diff(length_y, w_a);

length_z_p_d_x = diff(length_z, d_x);
length_z_p_d_y = diff(length_z, d_y);
length_z_p_d_t = diff(length_z, d_t);
length_z_p_w_a = diff(length_z, w_a);