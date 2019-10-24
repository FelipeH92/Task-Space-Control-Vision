%% UR5 Movement Test - VREP
% Author: Felipe Nascimento - April 2019
% V-REP Scene: UR5_Movement_Test

%% Initialization

clear all
close all
clc

disp('Program started');
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

if (clientID == -1)
    disp('Failed connecting to remote API server');
    vrep.delete();
    return;
end

disp('Connected to remote API server');

handles = struct('clientID', clientID);

% enable the synchronous mode on the client:
vrep.simxSynchronous(handles.clientID,true);

handles.UR5_joint = zeros(1,6); % six joints;

UR5JointArray = zeros(1,6);
UR5JointSpeed = zeros(1,6);
UR5PoseArray = zeros(1,6);

[~, handles.UR5] = vrep.simxGetObjectHandle(handles.clientID,'world_visual',vrep.simx_opmode_blocking);
[~, handles.UR5_Base] = vrep.simxGetObjectHandle(handles.clientID,'world_visual',vrep.simx_opmode_blocking);
[~, handles.UR5_EndEffector] = vrep.simxGetObjectHandle(handles.clientID,'ee_fixed_joint',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(1)] = vrep.simxGetObjectHandle(handles.clientID,'shoulder_pan_joint',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(2)] = vrep.simxGetObjectHandle(handles.clientID,'shoulder_lift_joint',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(3)] = vrep.simxGetObjectHandle(handles.clientID,'elbow_joint',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(4)] = vrep.simxGetObjectHandle(handles.clientID,'wrist_1_joint',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(5)] = vrep.simxGetObjectHandle(handles.clientID,'wrist_2_joint',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(6)] = vrep.simxGetObjectHandle(handles.clientID,'wrist_3_joint',vrep.simx_opmode_blocking);

% [~, handles.UR5] = vrep.simxGetObjectHandle(handles.clientID,'UR5',vrep.simx_opmode_blocking);
% [~, handles.UR5_Base] = vrep.simxGetObjectHandle(handles.clientID,'Frame_Base_1',vrep.simx_opmode_blocking);
% [~, handles.UR5_EndEffector] = vrep.simxGetObjectHandle(handles.clientID,'UR5_connection',vrep.simx_opmode_blocking);
% [~, handles.UR5_joint(1)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_joint1',vrep.simx_opmode_blocking);
% [~, handles.UR5_joint(2)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_joint2',vrep.simx_opmode_blocking);
% [~, handles.UR5_joint(3)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_joint3',vrep.simx_opmode_blocking);
% [~, handles.UR5_joint(4)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_joint4',vrep.simx_opmode_blocking);
% [~, handles.UR5_joint(5)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_joint5',vrep.simx_opmode_blocking);
% [~, handles.UR5_joint(6)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_joint6',vrep.simx_opmode_blocking);


% Simulation step(V-rep)
for k=1:6
    step = 10e-3;
    stepCount = 1;
    MaxSpeed = 2; % Rad/s 1,2,pi
    frequency = 1; % 0.5,1,2
    SimulationTime = 2/frequency;
    SimulationSteps = SimulationTime/step;

    UR5JointArray = zeros(1,6);
    UR5JointSpeed = zeros(1,6);
    UR5PoseArray = zeros(1,6);

    ur5_constant_matrices;
    UR5_JointSpeed = zeros(SimulationSteps,6);
    UR5_JointPosition = zeros(SimulationSteps,6);
    UR5_PoseArray = zeros(SimulationSteps,6);
    SimulationTimeMatrix = zeros(SimulationSteps,1);

%% Simulation Initialization

    vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
    vrep.simxSynchronous(handles.clientID,true);
    pause(5);

    [~,UR5JointArray(:)] = getUR5JointPositions(vrep,handles.clientID,handles.UR5_joint(:),vrep.simx_opmode_streaming);
    for i=1:6
        [~,UR5JointSpeed(i)] = vrep.simxGetObjectFloatParameter(handles.clientID,handles.UR5_joint(i), 2012, vrep.simx_opmode_streaming);
    end
    [~,UR5PoseArray(1:3)] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_EndEffector,handles.UR5_Base,vrep.simx_opmode_streaming);
    [~,UR5PoseArray(4:6)] = vrep.simxGetObjectOrientation(handles.clientID,handles.UR5_EndEffector,handles.UR5_Base,vrep.simx_opmode_streaming);



%% Simulation Start
    disp('Starting Simulation');
    while (stepCount <= SimulationSteps)

        vrep.simxPauseCommunication(handles.clientID, true);
        for j=1:6
            if j == k
                aux = 1;
            else
                aux = 0;
            end
            [~] = vrep.simxSetJointTargetVelocity(handles.clientID,handles.UR5_joint(j),(MaxSpeed*sin(2*pi*frequency*stepCount*step))*aux,vrep.simx_opmode_oneshot);
        end
        vrep.simxPauseCommunication(handles.clientID, false);

        vrep.simxSynchronousTrigger(handles.clientID);

        [~,UR5JointArray(:)] = getUR5JointPositions(vrep,handles.clientID,handles.UR5_joint(:),vrep.simx_opmode_buffer);
        for l=1:6
            [~,UR5JointSpeed(l)] = vrep.simxGetObjectFloatParameter(handles.clientID,handles.UR5_joint(l), 2012, vrep.simx_opmode_buffer);
        end
        [~,UR5PoseArray(1:3)] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_EndEffector,handles.UR5_Base,vrep.simx_opmode_buffer);
        [~,UR5PoseArray(4:6)] = vrep.simxGetObjectOrientation(handles.clientID,handles.UR5_EndEffector,handles.UR5_Base,vrep.simx_opmode_buffer);
        UR5_JointPosition(stepCount,:) = UR5JointArray(:);
        UR5_JointSpeed(stepCount,:) = UR5JointSpeed(:);
        UR5_PoseArray(stepCount,:) = UR5PoseArray;
        SimulationTimeMatrix(stepCount) = stepCount*step;

        stepCount = stepCount + 1;
    end

    [simState] = vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);
    pause(5);
%% Save Files

    filename = strcat('/home/nascimento/Projects/VREP/Scenes/Matlab-VREP/Figures/Movement Test/Joint Test/Joint',num2str(k));
    mkdir(filename);
    filename = strcat('A',num2str(MaxSpeed),'F',num2str(frequency),'S',num2str(SimulationTime));
    filename = strcat('/home/nascimento/Projects/VREP/Scenes/Matlab-VREP/Figures/Movement Test/Joint Test/Joint',num2str(k),'/',filename,'.mat');
    save(filename,'MaxSpeed','frequency','SimulationTimeMatrix','SimulationSteps','UR5_JointPosition','UR5_JointSpeed','UR5_PoseArray')
end

%% Plot
figure(1)
plot(SimulationTimeMatrix,UR5_JointPosition(:,1));
title('Posição da Junta 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('Posição (rad)');

figure(2)
plot(SimulationTimeMatrix,UR5_JointSpeed(:,1));
title('Velocidade de Junta 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('Posição (rad)');

figure(3)
subplot(3,1,1);
plot(SimulationTimeMatrix,UR5_PoseArray(:,1));
title('Posição do EndEffector - X');
grid on;
xlabel('Tempo de Simulação');
ylabel('X (m)');

subplot(3,1,2);
plot(SimulationTimeMatrix,UR5_PoseArray(:,2));
title('Posição do EndEffector - Y');
grid on;
xlabel('Tempo de Simulação');
ylabel('Y (m)');

subplot(3,1,3);
plot(SimulationTimeMatrix,UR5_PoseArray(:,3));
title('Posição do EndEffector - Z');
grid on;
xlabel('Tempo de Simulação');
ylabel('Z (m)');

%% Finalization

disp('Simulation ended');

vrep.simxFinish(handles.clientID);

vrep.delete();