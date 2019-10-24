%% UR5 Direct Kinematics

function H = ur5_direct_kinematics(q)

    if (exist('C_1') == 0)
        ur5_constant_matrices;
    end
    
    Rot_z_1 = [cos(q(1)), -sin(q(1)), 0, 0;
               sin(q(1)),  cos(q(1)), 0, 0;
               0, 0, 1, 0;
               0, 0, 0, 1];
    Rot_z_2 = [cos(q(2)), -sin(q(2)), 0, 0;
               sin(q(2)),  cos(q(2)), 0, 0;
               0, 0, 1, 0;
               0, 0, 0, 1];
    Rot_z_3 = [cos(q(3)), -sin(q(3)), 0, 0;
               sin(q(3)),  cos(q(3)), 0, 0;
               0, 0, 1, 0;
               0, 0, 0, 1];
    Rot_z_4 = [cos(q(4)), -sin(q(4)), 0, 0;
               sin(q(4)),  cos(q(4)), 0, 0;
               0, 0, 1, 0;
               0, 0, 0, 1];
    Rot_z_5 = [cos(q(5)), -sin(q(5)), 0, 0;
               sin(q(5)),  cos(q(5)), 0, 0;
               0, 0, 1, 0;
               0, 0, 0, 1];
    Rot_z_6 = [cos(q(6)), -sin(q(6)), 0, 0;
               sin(q(6)),  cos(q(6)), 0, 0;
               0, 0, 1, 0;
               0, 0, 0, 1]; 
           
    A_1 = Rot_z_1*C_1;
    A_2 = Rot_z_2*C_2;
    A_3 = Rot_z_3*C_3;
    A_4 = Rot_z_4*C_4;
    A_5 = Rot_z_5*C_5;
    A_6 = Rot_z_6*C_6;

    H = A_1*A_2*A_3*A_4*A_5*A_6;
    
end