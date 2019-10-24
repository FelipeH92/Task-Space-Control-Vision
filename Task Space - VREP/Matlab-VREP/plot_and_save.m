%% Erro Absoluto
figure(1);
subplot(2,1,1);
plot(SimulationTime,absErrorLeftDatabase);
line([SimulationTime(1),SimulationTime(end)],[trajectoryThreshold,trajectoryThreshold]);
title('Erro absoluto de posição - Braço 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('Erro absoluto')

subplot(2,1,2);
plot(SimulationTime,absErrorRightDatabase);
line([SimulationTime(1),SimulationTime(end)],[trajectoryThreshold,trajectoryThreshold]);
title('Erro absoluto de posição - Braço 2');
grid on;
xlabel('Tempo de Simulação');
ylabel('Erro absoluto')

saveas(gcf,'Erro_Absoluto_Posicao','svg');
%

%
figure(11);
plot(SimulationTime,absErrorLeftDatabase);
line([SimulationTime(1),SimulationTime(end)],[trajectoryThreshold,trajectoryThreshold]);
title('Erro absoluto de posição - Braço 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('Erro absoluto');

saveas(gcf,'Erro_Absoluto_Posicao_Braco_1','svg');
%

%
figure(12);
plot(SimulationTime,absErrorRightDatabase);
line([SimulationTime(1),SimulationTime(end)],[trajectoryThreshold,trajectoryThreshold]);
title('Erro absoluto de posição - Braço 2');
grid on;
xlabel('Tempo de Simulação');
ylabel('Erro absoluto')

saveas(gcf,'Erro_Absoluto_Posicao_Braco_2','svg');
%
%% Velocidade de juntas
figure(2);
subplot(6,2,1);
plot(SimulationTime,UR5_1_JointSpeed(:,1));
title('Velocidade de junta - Braço 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

subplot(6,2,3);
plot(SimulationTime,UR5_1_JointSpeed(:,2));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q2)');

subplot(6,2,5);
plot(SimulationTime,UR5_1_JointSpeed(:,3));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q3)');

subplot(6,2,7);
plot(SimulationTime,UR5_1_JointSpeed(:,4));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q4)');

subplot(6,2,9);
plot(SimulationTime,UR5_1_JointSpeed(:,5));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q5)');

subplot(6,2,11);
plot(SimulationTime,UR5_1_JointSpeed(:,6));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q6)');


subplot(6,2,2);
plot(SimulationTime,UR5_2_JointSpeed(:,1));
title('Velocidade de junta - Braço 2');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

subplot(6,2,4);
plot(SimulationTime,UR5_2_JointSpeed(:,2));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q2)');

subplot(6,2,6);
plot(SimulationTime,UR5_2_JointSpeed(:,3));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q3)');

subplot(6,2,8);
plot(SimulationTime,UR5_2_JointSpeed(:,4));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q4)');

subplot(6,2,10);
plot(SimulationTime,UR5_2_JointSpeed(:,5));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q5)');

subplot(6,2,12);
plot(SimulationTime,UR5_2_JointSpeed(:,6));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q6)');

saveas(gcf,'Velocidade_de_Junta','svg');
%

%
figure(5);
subplot(3,2,1);
plot(SimulationTime,UR5_1_JointSpeed(:,1));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

subplot(3,2,3);
plot(SimulationTime,UR5_1_JointSpeed(:,2));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q2)');

subplot(3,2,5);
plot(SimulationTime,UR5_1_JointSpeed(:,3));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q3)');

subplot(3,2,2);
plot(SimulationTime,UR5_1_JointSpeed(:,4));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q4)');

subplot(3,2,4);
plot(SimulationTime,UR5_1_JointSpeed(:,5));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q5)');

subplot(3,2,6);
plot(SimulationTime,UR5_1_JointSpeed(:,6));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q6)');

saveas(gcf,'Velocidade_de_junta_braco_1','svg');

%

figure(6);
subplot(3,2,1);
plot(SimulationTime,UR5_2_JointSpeed(:,1));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

subplot(3,2,3);
plot(SimulationTime,UR5_2_JointSpeed(:,2));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q2)');

subplot(3,2,5);
plot(SimulationTime,UR5_2_JointSpeed(:,3));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q3)');

subplot(3,2,2);
plot(SimulationTime,UR5_2_JointSpeed(:,4));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q4)');

subplot(3,2,4);
plot(SimulationTime,UR5_2_JointSpeed(:,5));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q5)');

subplot(3,2,6);
plot(SimulationTime,UR5_2_JointSpeed(:,6));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q6)');

saveas(gcf,'Velocidade_de_junta_braco_2','svg');
%

%
figure(13)
plot(SimulationTime,UR5_1_JointSpeed(:,1));
title('Velocidade de junta - Braço 1 - q1');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

saveas(gcf,'Velocidade_de_junta_braco_1_q1','svg');
%
%
figure(14)
plot(SimulationTime,UR5_1_JointSpeed(:,2));
title('Velocidade de junta - Braço 1 - q2');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

saveas(gcf,'Velocidade_de_junta_braco_1_q2','svg');
%
%
figure(15)
plot(SimulationTime,UR5_1_JointSpeed(:,3));
title('Velocidade de junta - Braço 1 - q3');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

saveas(gcf,'Velocidade_de_junta_braco_1_q3','svg');
%
%
figure(16)
plot(SimulationTime,UR5_1_JointSpeed(:,4));
title('Velocidade de junta - Braço 1 - q4');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

saveas(gcf,'Velocidade_de_junta_braco_1_q4','svg');
%
%
figure(17)
plot(SimulationTime,UR5_1_JointSpeed(:,5));
title('Velocidade de junta - Braço 1 - q5');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

saveas(gcf,'Velocidade_de_junta_braco_1_q5','svg');
%
%
figure(18)
plot(SimulationTime,UR5_1_JointSpeed(:,6));
title('Velocidade de junta - Braço 1 - q6');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

saveas(gcf,'Velocidade_de_junta_braco_1_q6','svg');
%
%
figure(19)
plot(SimulationTime,UR5_2_JointSpeed(:,1));
title('Velocidade de junta - Braço 2 - q1');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

saveas(gcf,'Velocidade_de_junta_braco_2_q1','svg');
%
%
figure(20)
plot(SimulationTime,UR5_2_JointSpeed(:,2));
title('Velocidade de junta - Braço 2 - q2');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

saveas(gcf,'Velocidade_de_junta_braco_2_q2','svg');
%
%
figure(21)
plot(SimulationTime,UR5_2_JointSpeed(:,3));
title('Velocidade de junta - Braço 2 - q3');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

saveas(gcf,'Velocidade_de_junta_braco_2_q3','svg');
%
%
figure(22)
plot(SimulationTime,UR5_2_JointSpeed(:,4));
title('Velocidade de junta - Braço 2 - q4');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

saveas(gcf,'Velocidade_de_junta_braco_2_q4','svg');
%
%
figure(23)
plot(SimulationTime,UR5_2_JointSpeed(:,5));
title('Velocidade de junta - Braço 2 - q5');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

saveas(gcf,'Velocidade_de_junta_braco_2_q5','svg');
%
%
figure(24)
plot(SimulationTime,UR5_2_JointSpeed(:,6));
title('Velocidade de junta - Braço 2 - q6');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

saveas(gcf,'Velocidade_de_junta_braco_2_q6','svg');
%


%% Erro de Posição e Rotação
figure(3);
subplot(6,2,1);
plot(SimulationTime,Trajectory_UR5_1(:,1)',SimulationTime,Trajectory_UR5_1_desired(:,1)',SimulationTime,ErroPosition_UR5_1(:,1)');
title('Erro de Pose - Braço 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('X');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

subplot(6,2,3);
plot(SimulationTime,Trajectory_UR5_1(:,2)',SimulationTime,Trajectory_UR5_1_desired(:,2)',SimulationTime,ErroPosition_UR5_1(:,2)');
grid on;
xlabel('Tempo de Simulação');
ylabel('Y');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');


subplot(6,2,5);
plot(SimulationTime,Trajectory_UR5_1(:,3)',SimulationTime,Trajectory_UR5_1_desired(:,3)',SimulationTime,ErroPosition_UR5_1(:,3)');
grid on;
xlabel('Tempo de Simulação');
ylabel('Z');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

subplot(6,2,7);
plot(SimulationTime,Trajectory_UR5_1(:,4)',SimulationTime,Trajectory_UR5_1_desired(:,4)',SimulationTime,ErroPosition_UR5_1(:,4)');
grid on;
xlabel('Tempo de Simulação');
ylabel('R(x)');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

subplot(6,2,9);
plot(SimulationTime,Trajectory_UR5_1(:,5)',SimulationTime,Trajectory_UR5_1_desired(:,5)',SimulationTime,ErroPosition_UR5_1(:,5)');
grid on;
xlabel('Tempo de Simulação');
ylabel('R(y)');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

subplot(6,2,11);
plot(SimulationTime,Trajectory_UR5_1(:,6)',SimulationTime,Trajectory_UR5_1_desired(:,6)',SimulationTime,ErroPosition_UR5_1(:,6)');
grid on;
xlabel('Tempo de Simulação');
ylabel('R(z)');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');



subplot(6,2,2);
plot(SimulationTime,Trajectory_UR5_2(:,1)',SimulationTime,Trajectory_UR5_2_desired(:,1)',SimulationTime,ErroPosition_UR5_2(:,1)');
title('Erro de Pose - Braço 2');
grid on;
xlabel('Tempo de Simulação');
ylabel('X');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

subplot(6,2,4);
plot(SimulationTime,Trajectory_UR5_2(:,2)',SimulationTime,Trajectory_UR5_2_desired(:,2)',SimulationTime,ErroPosition_UR5_2(:,2)');
grid on;
xlabel('Tempo de Simulação');
ylabel('Y');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

subplot(6,2,6);
plot(SimulationTime,Trajectory_UR5_2(:,3)',SimulationTime,Trajectory_UR5_2_desired(:,3)',SimulationTime,ErroPosition_UR5_2(:,3)');
grid on;
xlabel('Tempo de Simulação');
ylabel('Z');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

subplot(6,2,8);
plot(SimulationTime,Trajectory_UR5_2(:,4)',SimulationTime,Trajectory_UR5_2_desired(:,4)',SimulationTime,ErroPosition_UR5_2(:,4)');
grid on;
xlabel('Tempo de Simulação');
ylabel('R(x)');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

subplot(6,2,10);
plot(SimulationTime,Trajectory_UR5_2(:,5)',SimulationTime,Trajectory_UR5_2_desired(:,5)',SimulationTime,ErroPosition_UR5_2(:,5)');
grid on;
xlabel('Tempo de Simulação');
ylabel('R(y)');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

subplot(6,2,12);
plot(SimulationTime,Trajectory_UR5_2(:,6)',SimulationTime,Trajectory_UR5_2_desired(:,6)',SimulationTime,ErroPosition_UR5_2(:,6)');
grid on;
xlabel('Tempo de Simulação');
ylabel('R(z)');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

saveas(gcf,'Trajetoria','svg');
%

%
figure(7);
subplot(3,2,1);
plot(SimulationTime,Trajectory_UR5_1(:,1)',SimulationTime,Trajectory_UR5_1_desired(:,1)',SimulationTime,ErroPosition_UR5_1(:,1)');
grid on;
xlabel('Tempo de Simulação');
ylabel('X');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

subplot(3,2,3);
plot(SimulationTime,Trajectory_UR5_1(:,2)',SimulationTime,Trajectory_UR5_1_desired(:,2)',SimulationTime,ErroPosition_UR5_1(:,2)');
grid on;
xlabel('Tempo de Simulação');
ylabel('Y');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');


subplot(3,2,5);
plot(SimulationTime,Trajectory_UR5_1(:,3)',SimulationTime,Trajectory_UR5_1_desired(:,3)',SimulationTime,ErroPosition_UR5_1(:,3)');
grid on;
xlabel('Tempo de Simulação');
ylabel('Z');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

subplot(3,2,2);
plot(SimulationTime,Trajectory_UR5_1(:,4)',SimulationTime,Trajectory_UR5_1_desired(:,4)',SimulationTime,ErroPosition_UR5_1(:,4)');
grid on;
xlabel('Tempo de Simulação');
ylabel('R(x)');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

subplot(3,2,4);
plot(SimulationTime,Trajectory_UR5_1(:,5)',SimulationTime,Trajectory_UR5_1_desired(:,5)',SimulationTime,ErroPosition_UR5_1(:,5)');
grid on;
xlabel('Tempo de Simulação');
ylabel('R(y)');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

subplot(3,2,6);
plot(SimulationTime,Trajectory_UR5_1(:,6)',SimulationTime,Trajectory_UR5_1_desired(:,6)',SimulationTime,ErroPosition_UR5_1(:,6)');
grid on;
xlabel('Tempo de Simulação');
ylabel('R(z)');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

saveas(gcf,'Trajetoria_braco_1','svg');
%

figure(8)
subplot(3,2,1);
plot(SimulationTime,Trajectory_UR5_2(:,1)',SimulationTime,Trajectory_UR5_2_desired(:,1)',SimulationTime,ErroPosition_UR5_2(:,1)');
grid on;
xlabel('Tempo de Simulação');
ylabel('X');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

subplot(3,2,3);
plot(SimulationTime,Trajectory_UR5_2(:,2)',SimulationTime,Trajectory_UR5_2_desired(:,2)',SimulationTime,ErroPosition_UR5_2(:,2)');
grid on;
xlabel('Tempo de Simulação');
ylabel('Y');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

subplot(3,2,5);
plot(SimulationTime,Trajectory_UR5_2(:,3)',SimulationTime,Trajectory_UR5_2_desired(:,3)',SimulationTime,ErroPosition_UR5_2(:,3)');
grid on;
xlabel('Tempo de Simulação');
ylabel('Z');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

subplot(3,2,2);
plot(SimulationTime,Trajectory_UR5_2(:,4)',SimulationTime,Trajectory_UR5_2_desired(:,4)',SimulationTime,ErroPosition_UR5_2(:,4)');
grid on;
xlabel('Tempo de Simulação');
ylabel('R(x)');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

subplot(3,2,4);
plot(SimulationTime,Trajectory_UR5_2(:,5)',SimulationTime,Trajectory_UR5_2_desired(:,5)',SimulationTime,ErroPosition_UR5_2(:,5)');
grid on;
xlabel('Tempo de Simulação');
ylabel('R(y)');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

subplot(3,2,6);
plot(SimulationTime,Trajectory_UR5_2(:,6)',SimulationTime,Trajectory_UR5_2_desired(:,6)',SimulationTime,ErroPosition_UR5_2(:,6)');
grid on;
xlabel('Tempo de Simulação');
ylabel('R(z)');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

saveas(gcf,'Trajetoria_braco_2','svg');
%

%
figure(25)
plot(SimulationTime,Trajectory_UR5_1(:,1)',SimulationTime,Trajectory_UR5_1_desired(:,1)',SimulationTime,ErroPosition_UR5_1(:,1)');
title('Erro de Pose - Braço 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('X');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

saveas(gcf,'Trajetoria_braco_1_x','svg');
%
%
figure(26)
plot(SimulationTime,Trajectory_UR5_1(:,2)',SimulationTime,Trajectory_UR5_1_desired(:,2)',SimulationTime,ErroPosition_UR5_1(:,2)');
title('Erro de Pose - Braço 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('Y');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

saveas(gcf,'Trajetoria_braco_1_y','svg');
%
%
figure(27)
plot(SimulationTime,Trajectory_UR5_1(:,3)',SimulationTime,Trajectory_UR5_1_desired(:,3)',SimulationTime,ErroPosition_UR5_1(:,3)');
title('Erro de Pose - Braço 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('Z');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

saveas(gcf,'Trajetoria_braco_1_z','svg');
%
%
figure(28)
plot(SimulationTime,Trajectory_UR5_1(:,4)',SimulationTime,Trajectory_UR5_1_desired(:,4)',SimulationTime,ErroPosition_UR5_1(:,4)');
title('Erro de Pose - Braço 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('R(x) - Radianos');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

saveas(gcf,'Trajetoria_braco_1_rx','svg');
%
%
figure(29)
plot(SimulationTime,Trajectory_UR5_1(:,5)',SimulationTime,Trajectory_UR5_1_desired(:,5)',SimulationTime,ErroPosition_UR5_1(:,5)');
title('Erro de Pose - Braço 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('R(y) - Radianos');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

saveas(gcf,'Trajetoria_braco_1_ry','svg');
%
%
figure(30)
plot(SimulationTime,Trajectory_UR5_1(:,6)',SimulationTime,Trajectory_UR5_1_desired(:,6)',SimulationTime,ErroPosition_UR5_1(:,6)');
title('Erro de Pose - Braço 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('R(z) - Radianos');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

saveas(gcf,'Trajetoria_braco_1_rz','svg');
%


%
figure(31)
plot(SimulationTime,Trajectory_UR5_2(:,1)',SimulationTime,Trajectory_UR5_2_desired(:,1)',SimulationTime,ErroPosition_UR5_2(:,1)');
title('Erro de Pose - Braço 2');
grid on;
xlabel('Tempo de Simulação');
ylabel('X');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

saveas(gcf,'Trajetoria_braco_2_x','svg');
%
%
figure(32)
plot(SimulationTime,Trajectory_UR5_2(:,2)',SimulationTime,Trajectory_UR5_2_desired(:,2)',SimulationTime,ErroPosition_UR5_2(:,2)');
title('Erro de Pose - Braço 2');
grid on;
xlabel('Tempo de Simulação');
ylabel('Y');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

saveas(gcf,'Trajetoria_braco_2_y','svg');
%
%
figure(33)
plot(SimulationTime,Trajectory_UR5_2(:,3)',SimulationTime,Trajectory_UR5_2_desired(:,3)',SimulationTime,ErroPosition_UR5_2(:,3)');
title('Erro de Pose - Braço 2');
grid on;
xlabel('Tempo de Simulação');
ylabel('Z');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

saveas(gcf,'Trajetoria_braco_2_z','svg');
%
%
figure(34)
plot(SimulationTime,Trajectory_UR5_2(:,4)',SimulationTime,Trajectory_UR5_2_desired(:,4)',SimulationTime,ErroPosition_UR5_2(:,4)');
title('Erro de Pose - Braço 2');
grid on;
xlabel('Tempo de Simulação');
ylabel('R(x)');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

saveas(gcf,'Trajetoria_braco_2_rx','svg');
%
%
figure(35)
plot(SimulationTime,Trajectory_UR5_2(:,5)',SimulationTime,Trajectory_UR5_2_desired(:,5)',SimulationTime,ErroPosition_UR5_2(:,5)');
title('Erro de Pose - Braço 2');
grid on;
xlabel('Tempo de Simulação');
ylabel('R(y)');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

saveas(gcf,'Trajetoria_braco_2_ry','svg');
%
%
figure(36)
plot(SimulationTime,Trajectory_UR5_2(:,6)',SimulationTime,Trajectory_UR5_2_desired(:,6)',SimulationTime,ErroPosition_UR5_2(:,6)');
title('Erro de Pose - Braço 2');
grid on;
xlabel('Tempo de Simulação');
ylabel('R(z)');
legend('Trajetória realizada', 'Trajetória desejada', 'Erro');

saveas(gcf,'Trajetoria_braco_2_rz','svg');
%
%% Posição de juntas
%
figure(4);
subplot(6,2,1);
plot(SimulationTime,UR5_1_Position(:,1)');
title('Posição de junta - Braço 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('q1');

subplot(6,2,3);
plot(SimulationTime,UR5_1_Position(:,2)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q2');

subplot(6,2,5);
plot(SimulationTime,UR5_1_Position(:,3)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q3');

subplot(6,2,7);
plot(SimulationTime,UR5_1_Position(:,4)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q4');

subplot(6,2,9);
plot(SimulationTime,UR5_1_Position(:,5)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q5');

subplot(6,2,11);
plot(SimulationTime,UR5_1_Position(:,6)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q6');



subplot(6,2,2);
plot(SimulationTime,UR5_2_Position(:,1)');
title('Posição de junta - Braço 2');
grid on;
xlabel('Tempo de Simulação');
ylabel('q1');

subplot(6,2,4);
plot(SimulationTime,UR5_2_Position(:,2)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q2');

subplot(6,2,6);
plot(SimulationTime,UR5_2_Position(:,3)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q3');

subplot(6,2,8);
plot(SimulationTime,UR5_2_Position(:,4)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q4');

subplot(6,2,10);
plot(SimulationTime,UR5_2_Position(:,5)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q5');

subplot(6,2,12);
plot(SimulationTime,UR5_2_Position(:,6)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q6');

saveas(gcf,'Posicao_de_Junta','svg');
%

%
figure(9);
subplot(3,2,1);
plot(SimulationTime,UR5_1_Position(:,1)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q1');

subplot(3,2,3);
plot(SimulationTime,UR5_1_Position(:,2)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q2');

subplot(3,2,5);
plot(SimulationTime,UR5_1_Position(:,3)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q3');

subplot(3,2,2);
plot(SimulationTime,UR5_1_Position(:,4)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q4');

subplot(3,2,4);
plot(SimulationTime,UR5_1_Position(:,5)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q5');

subplot(3,2,6);
plot(SimulationTime,UR5_1_Position(:,6)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q6');

saveas(gcf,'Posicao_Junta_Braco_1','svg');

%

%

figure(10);
subplot(3,2,1);
plot(SimulationTime,UR5_2_Position(:,1)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q1');

subplot(3,2,3);
plot(SimulationTime,UR5_2_Position(:,2)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q2');

subplot(3,2,5);
plot(SimulationTime,UR5_2_Position(:,3)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q3');

subplot(3,2,2);
plot(SimulationTime,UR5_2_Position(:,4)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q4');

subplot(3,2,4);
plot(SimulationTime,UR5_2_Position(:,5)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q5');

subplot(3,2,6);
plot(SimulationTime,UR5_2_Position(:,6)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q6');

saveas(gcf,'Posicao_Junta_Braco_2','svg');

%

%
figure(37)
plot(SimulationTime,UR5_1_Position(:,1)');
title('Posição de junta - Braço 1 - q1');
grid on;
xlabel('Tempo de Simulação');
ylabel('q1');

saveas(gcf,'Posicao_Junta_Braco_1_q1','svg');
%
%
figure(38)
plot(SimulationTime,UR5_1_Position(:,2)');
title('Posição de junta - Braço 1 - q2');
grid on;
xlabel('Tempo de Simulação');
ylabel('q2');

saveas(gcf,'Posicao_Junta_Braco_1_q2','svg');
%
%
figure(39)
plot(SimulationTime,UR5_1_Position(:,3)');
title('Posição de junta - Braço 1 - q3');
grid on;
xlabel('Tempo de Simulação');
ylabel('q3');

saveas(gcf,'Posicao_Junta_Braco_1_q3','svg');
%
%
figure(40)
plot(SimulationTime,UR5_1_Position(:,4)');
title('Posição de junta - Braço 1 - q4');
grid on;
xlabel('Tempo de Simulação');
ylabel('q4');

saveas(gcf,'Posicao_Junta_Braco_1_q4','svg');
%
%
figure(41)
plot(SimulationTime,UR5_1_Position(:,5)');
title('Posição de junta - Braço 1 - q5');
grid on;
xlabel('Tempo de Simulação');
ylabel('q5');

saveas(gcf,'Posicao_Junta_Braco_1_q5','svg');
%
%
figure(42)
plot(SimulationTime,UR5_1_Position(:,6)');
title('Posição de junta - Braço 1 - q6');
grid on;
xlabel('Tempo de Simulação');
ylabel('q6');

saveas(gcf,'Posicao_Junta_Braco_1_q6','svg');
%
%
figure(43)
plot(SimulationTime,UR5_2_Position(:,1)');
title('Posição de junta - Braço 2 - q1');
grid on;
xlabel('Tempo de Simulação');
ylabel('q1');

saveas(gcf,'Posicao_Junta_Braco_2_q1','svg');
%
%
figure(44)
plot(SimulationTime,UR5_2_Position(:,2)');
title('Posição de junta - Braço 2 - q2');
grid on;
xlabel('Tempo de Simulação');
ylabel('q2');

saveas(gcf,'Posicao_Junta_Braco_2_q2','svg');
%
%
figure(45)
plot(SimulationTime,UR5_2_Position(:,3)');
title('Posição de junta - Braço 2 - q3');
grid on;
xlabel('Tempo de Simulação');
ylabel('q3');

saveas(gcf,'Posicao_Junta_Braco_2_q3','svg');
%
%
figure(46)
plot(SimulationTime,UR5_2_Position(:,4)');
title('Posição de junta - Braço 2 - q4');
grid on;
xlabel('Tempo de Simulação');
ylabel('q4');

saveas(gcf,'Posicao_Junta_Braco_2_q4','svg');
%
%
figure(47)
plot(SimulationTime,UR5_2_Position(:,5)');
title('Posição de junta - Braço 2 - q5');
grid on;
xlabel('Tempo de Simulação');
ylabel('q5');

saveas(gcf,'Posicao_Junta_Braco_2_q5','svg');
%
%
figure(48)
plot(SimulationTime,UR5_2_Position(:,6)');
title('Posição de junta - Braço 2 - q6');
grid on;
xlabel('Tempo de Simulação');
ylabel('q6');

saveas(gcf,'Posicao_Junta_Braco_2_q6','svg');
%
