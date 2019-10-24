function theta = eulerzyx_to_theta(rotation_matrix )
% EULERZYXINV - Long Qian - Modified by: Felipe Nascimento
%	The rotational matrix can be represented by:
%	R = Rx(t1) * Ry(t2) * Rz(t3)
%   theta - inverse eulerzyx angle. 1*3 vector
    theta = [0,0,0];
    % If the rotational matrix represents a singularity
    if abs(rotation_matrix(3,3)) < eps && abs(rotation_matrix(2,3)) < eps
        theta(1) = 0;
        theta(2) = atan2(rotation_matrix(1,3), rotation_matrix(3,3));
        theta(3) = atan2(rotation_matrix(2,1), rotation_matrix(2,2));
    % Normal case
    else
        theta(1) = atan2(-rotation_matrix(2,3), rotation_matrix(3,3));
        sinr = sin(theta(1));
        cosr = cos(theta(1));
        theta(2) = atan2(rotation_matrix(1,3), cosr * rotation_matrix(3,3) - sinr * rotation_matrix(2,3));
        theta(3) = atan2(-rotation_matrix(1,2), rotation_matrix(1,1));
    end
end