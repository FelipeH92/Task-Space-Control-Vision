% Save Images Dissertação

figure(1);
plot(SimulationTime,absErrorLeftDatabase);
title('Erro Absoluto do UR5');
grid on;
xlabel('Tempo de Simulação (s)');
ylabel('Posição (m)');
%xlim([0 10]);
ylim([0 0.02]);

figure(2)
plot(SimulationTime,absErrorLeftRotDatabase);
title('Erro Absoluto do UR5');
grid on;
xlabel('Tempo de Simulação (s)');
ylabel('Rotação (rad)');

fc = 3;
fs = 100;
[b,a] = butter(6,fc/(fs/2));

UR5_1_JointSpeedOut = UR5_1_JointSpeed;

UR5_1_JointSpeedOut(:,1) = filter(b,a,UR5_1_JointSpeed(:,1));
UR5_1_JointSpeedOut(:,2) = filter(b,a,UR5_1_JointSpeed(:,2));
UR5_1_JointSpeedOut(:,3) = filter(b,a,UR5_1_JointSpeed(:,3));
UR5_1_JointSpeedOut(:,4) = filter(b,a,UR5_1_JointSpeed(:,4));
UR5_1_JointSpeedOut(:,5) = filter(b,a,UR5_1_JointSpeed(:,5));
UR5_1_JointSpeedOut(:,6) = filter(b,a,UR5_1_JointSpeed(:,6));

figure(3);
plot(SimulationTime,UR5_1_JointSpeedOut(:,1),SimulationTime,UR5_1_JointSpeedOut(:,2),SimulationTime,UR5_1_JointSpeedOut(:,3),SimulationTime,UR5_1_JointSpeedOut(:,4),SimulationTime,UR5_1_JointSpeedOut(:,5),SimulationTime,UR5_1_JointSpeedOut(:,6));
title('Velocidade de Junta');
grid on;
xlabel('Tempo de Simulação (s)');
ylabel('Velocidade (rad/s)');
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'location', 'northwest');
%xlim([0 10]);
ylim([-0.25 0.25]);

figure(4);
plot(SimulationTime,UR5_1_JointPosition(:,1)',SimulationTime,UR5_1_JointPosition(:,2)',SimulationTime,UR5_1_JointPosition(:,3)',SimulationTime,UR5_1_JointPosition(:,4)',SimulationTime,UR5_1_JointPosition(:,5)',SimulationTime,UR5_1_JointPosition(:,6)');
title('Posição de Junta')
grid on;
xlabel('Tempo de Simulação (s)');
ylabel('Posição (rad)');
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
%xlim([0 10]);
ylim([-pi pi]);
%%

figure(5);
plot3(Trajectory_UR5_1_desired(:,1),Trajectory_UR5_1_desired(:,2),Trajectory_UR5_1_desired(:,3))
hold on
plot3(Trajectory_UR5_1_desired(1,1),Trajectory_UR5_1_desired(1,2),Trajectory_UR5_1_desired(1,3),'ro')
title('Trajetória Planejada')
grid on;
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
legend('Trajetória','Posição inicial','Location','northeast');
%%
filename = '/home/nascimento/Projects/VREP/Scenes/Matlab-VREP/Figures/Controle/ControleProporcionalIntegral/';
mkdir(filename);
filename = strcat(filename,'/K5I025S05.mat');
%save(filename,'SimulationTime','ErroObjectDatabase','ObjectTrajectory','absErrorLeftDatabase','absErrorRightDatabase','absErrorLeftRotDatabase','absErrorRightRotDatabase','UR5_1_JointSpeed','UR5_2_JointSpeed','UR5_1_JointPosition','UR5_2_JointPosition','ErroPosition_UR5_1','ErroPosition_UR5_2','Trajectory_UR5_1','Trajectory_UR5_2','Trajectory_UR5_1_desired','Trajectory_UR5_2_desired')

fig = figure(1);
print(fig,'-dpng')
fig = figure(2);
print(fig,'-dpng')
fig = figure(3);
print(fig,'-dpng')
fig = figure(4);
print(fig,'-dpng')