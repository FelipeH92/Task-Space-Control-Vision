% Save Images Dissertação

figure(1);
plot(SimulationTime,absErrorLeftDatabase);
title('Erro Absoluto do UR5');
grid on;
xlabel('Tempo de Simulação');
ylabel('Posição');

figure(2);
plot(SimulationTime,UR5_1_JointSpeed(:,1),SimulationTime,UR5_1_JointSpeed(:,2),SimulationTime,UR5_1_JointSpeed(:,3),SimulationTime,UR5_1_JointSpeed(:,4),SimulationTime,UR5_1_JointSpeed(:,5),SimulationTime,UR5_1_JointSpeed(:,6));
title('Velocidade de Junta');
grid on;
xlabel('Tempo de Simulação');
ylabel('Velocidade (rad/s)');
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
xlim([0 10]);
ylim([-0.2 0.2]);

figure(3);
plot(SimulationTime,UR5_1_JointPosition(:,1)',SimulationTime,UR5_1_JointPosition(:,2)',SimulationTime,UR5_1_JointPosition(:,3)',SimulationTime,UR5_1_JointPosition(:,4)',SimulationTime,UR5_1_JointPosition(:,5)',SimulationTime,UR5_1_JointPosition(:,6)');
title('Posição de Junta')
grid on;
xlabel('Tempo de Simulação');
ylabel('Posição (rad)');
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
xlim([0 10]);
ylim([-pi pi]);

figure(4);
plot(SimulationTime,Trajectory_UR5_1(:,3)',SimulationTime,PoseToGetTo(3)*ones(1,998)');
title('Coordenada Z')
grid on;
xlabel('Tempo de Simulação');
ylabel('Z (m)');
legend('Trajetória realizada', 'Trajetória desejada','Location','southeast');

filename = '/home/nascimento/Projects/VREP/Scenes/Matlab-VREP/Figures/Degrau/';
mkdir(filename);
filename = strcat(filename,'/PINV.mat');
save(filename,'SimulationTime','ErroObjectDatabase','ObjectTrajectory','absErrorLeftDatabase','absErrorRightDatabase','absErrorLeftRotDatabase','absErrorRightRotDatabase','UR5_1_JointSpeed','UR5_2_JointSpeed','UR5_1_JointPosition','UR5_2_JointPosition','ErroPosition_UR5_1','ErroPosition_UR5_2','Trajectory_UR5_1','Trajectory_UR5_2','Trajectory_UR5_1_desired','Trajectory_UR5_2_desired')

fig = figure(1);
print(fig,'-dpng')
fig = figure(2);
print(fig,'-dpng')
fig = figure(3);
print(fig,'-dpng')
fig = figure(4);
print(fig,'-dpng')