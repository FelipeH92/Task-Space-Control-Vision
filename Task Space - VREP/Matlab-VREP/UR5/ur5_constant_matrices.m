%% UR5 generate constant matrices

standard_DH = [0,-.425,-.39225,0,0,0; 
              pi/2, 0, 0, pi/2, -pi/2, 0;
              .089159, 0, 0, .10915, .09465, .0823;
              0, 0, 0, 0, 0, 0];
          
% standard_DH = [0,0.005,0.4251,0.3922,0.0059,0.1007; 
%               pi/2, pi/2, -pi/2, 0, -pi/2, -pi/2;
%               .089159, 0.0661, 0, 0, 0.1150, 0.0;
%               0, 0, 0, 0, 0, 0];
        

a = standard_DH(1,:);
alpha = standard_DH(2,:);
d = standard_DH(3,:);

Rot_x_1 = [1, 0, 0, 0;
           0, cos(alpha(1)), -sin(alpha(1)), 0;
           0, sin(alpha(1)),  cos(alpha(1)), 0;
           0, 0, 0, 1];
Rot_x_2 = [1, 0, 0, 0;
           0, cos(alpha(2)), -sin(alpha(2)), 0;
           0, sin(alpha(2)),  cos(alpha(2)), 0;
           0, 0, 0, 1];
Rot_x_3 = [1, 0, 0, 0;
           0, cos(alpha(3)), -sin(alpha(3)), 0;
           0, sin(alpha(3)),  cos(alpha(3)), 0;
           0, 0, 0, 1];
Rot_x_4 = [1, 0, 0, 0;
           0, cos(alpha(4)), -sin(alpha(4)), 0;
           0, sin(alpha(4)),  cos(alpha(4)), 0;
           0, 0, 0, 1];
Rot_x_5 = [1, 0, 0, 0;
           0, cos(alpha(5)), -sin(alpha(5)), 0;
           0, sin(alpha(5)),  cos(alpha(5)), 0;
           0, 0, 0, 1];
Rot_x_6 = [1, 0, 0, 0;
           0, cos(alpha(6)), -sin(alpha(6)), 0;
           0, sin(alpha(6)),  cos(alpha(6)), 0;
           0, 0, 0, 1];


Trans_d_1 = [1, 0, 0, 0;
             0, 1, 0, 0;
             0, 0, 1, d(1);
             0, 0, 0, 1;];
Trans_d_2 = [1, 0, 0, 0;
             0, 1, 0, 0;
             0, 0, 1, d(2);
             0, 0, 0, 1;];
Trans_d_3 = [1, 0, 0, 0;
             0, 1, 0, 0;
             0, 0, 1, d(3);
             0, 0, 0, 1;];
Trans_d_4 = [1, 0, 0, 0;
             0, 1, 0, 0;
             0, 0, 1, d(4);
             0, 0, 0, 1;];
Trans_d_5 = [1, 0, 0, 0;
             0, 1, 0, 0;
             0, 0, 1, d(5);
             0, 0, 0, 1;];
Trans_d_6 = [1, 0, 0, 0;
             0, 1, 0, 0;
             0, 0, 1, d(6);
             0, 0, 0, 1;];



Trans_a_1 = [1, 0, 0, a(1);
             0, 1, 0, 0;
             0, 0, 1, 0;
             0, 0, 0, 1;];
Trans_a_2 = [1, 0, 0, a(2);
             0, 1, 0, 0;
             0, 0, 1, 0;
             0, 0, 0, 1;];
Trans_a_3 = [1, 0, 0, a(3);
             0, 1, 0, 0;
             0, 0, 1, 0;
             0, 0, 0, 1;];
Trans_a_4 = [1, 0, 0, a(4);
             0, 1, 0, 0;
             0, 0, 1, 0;
             0, 0, 0, 1;];
Trans_a_5 = [1, 0, 0, a(5);
             0, 1, 0, 0;
             0, 0, 1, 0;
             0, 0, 0, 1;];
Trans_a_6 = [1, 0, 0, a(6);
             0, 1, 0, 0;
             0, 0, 1, 0;
             0, 0, 0, 1;];
         
C_1 = Trans_d_1*Trans_a_1*Rot_x_1;
C_2 = Trans_d_2*Trans_a_2*Rot_x_2;
C_3 = Trans_d_3*Trans_a_3*Rot_x_3;
C_4 = Trans_d_4*Trans_a_4*Rot_x_4;
C_5 = Trans_d_5*Trans_a_5*Rot_x_5;
C_6 = Trans_d_6*Trans_a_6*Rot_x_6;