% Save Images Dissertação

figure(1);
plot(expTime,normErrorPosition_EXP);
title('Erro Absoluto do UR5');
grid on;
xlabel('Tempo de Experimento');
ylabel('Posição (m)');
xlim([0.008 10]);
%ylim([0 0.01]);

%
figure(2);
plot(expTime,PoseArray(:,3)',expTime,(PoseArray(1,3)+0.1)*ones(1,1250)');
title('Coordenada Z')
grid on;
xlabel('Tempo de Simulação');
ylabel('Z (m)');
legend('Trajetória realizada', 'Trajetória desejada','Location','southeast');
xlim([0.008 10]);
%

% figure(2)
% plot(expTime,normErrorRotation_EXP);
% title('Erro Absoluto do UR5');
% grid on;
% xlabel('Tempo de Experimento');
% ylabel('Rotação (rad)');

figure(3);
plot(expTime,speedListJointArray_EXP(:,1),expTime,speedListJointArray_EXP(:,2),expTime,speedListJointArray_EXP(:,3),expTime,speedListJointArray_EXP(:,4),expTime,speedListJointArray_EXP(:,5),expTime,speedListJointArray_EXP(:,6));
title('Velocidade de Junta');
grid on;
xlabel('Tempo de Experimento');
ylabel('Velocidade (rad/s)');
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'location', 'northeast');
xlim([0.008 10]);
ylim([-0.2 0.2]);

figure(4);
plot(expTime,jointPositionListArray(:,1),expTime,jointPositionListArray(:,2),expTime,jointPositionListArray(:,3),expTime,jointPositionListArray(:,4),expTime,jointPositionListArray(:,5),expTime,jointPositionListArray(:,6));
title('Posição de Junta')
grid on;
xlabel('Tempo de experimento');
ylabel('Posição (rad)');
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
xlim([0.008 10]);
ylim([-pi pi]);

% filename = '/home/nascimento/Projects/VREP/Scenes/Matlab-VREP/Figures/Controle/ControleProporcionalIntegral/';
% mkdir(filename);
% filename = strcat(filename,'/K5I025S05.mat');
% save(filename,'SimulationTime','ErroObjectDatabase','ObjectTrajectory','absErrorLeftDatabase','absErrorRightDatabase','absErrorLeftRotDatabase','absErrorRightRotDatabase','UR5_1_JointSpeed','UR5_2_JointSpeed','UR5_1_JointPosition','UR5_2_JointPosition','ErroPosition_UR5_1','ErroPosition_UR5_2','Trajectory_UR5_1','Trajectory_UR5_2','Trajectory_UR5_1_desired','Trajectory_UR5_2_desired')

fig = figure(1);
print(fig,'-dpng')
fig = figure(2);
print(fig,'-dpng')
fig = figure(3);
print(fig,'-dpng')
fig = figure(4);
print(fig,'-dpng')