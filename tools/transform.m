%%Create homogeneous transformation matrix from D-H parameters
function T = transform(a_i_min_1, alpha_i_min_1, d_i, theta_i)
T  = [cosd(theta_i)                     -sind(theta_i)                    0                    a_i_min_1;
      sind(theta_i)*cos(alpha_i_min_1)  cosd(theta_i)*cosd(alpha_i_min_1) -sind(alpha_i_min_1) -sind(alpha_i_min_1)*d_i;
      sind(theta_i)*sind(alpha_i_min_1) cosd(theta_i)*sind(alpha_i_min_1) cosd(alpha_i_min_1)  cosd(alpha_i_min_1)*d_i;
      0                                 0                                 0                    1];
