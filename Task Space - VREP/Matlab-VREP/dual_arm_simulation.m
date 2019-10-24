%% Dual Arm Position Control - VREP
% Author: Felipe Nascimento - January 2019
% V-REP Scene: Dual_UR5

%% Initialization

clear all
close all
clc

disp('Program started');
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
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
vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
vrep.simxSynchronous(handles.clientID,true);
pause(5);

% getting handles:
[~, handles.cuboid] = vrep.simxGetObjectHandle(handles.clientID,'Cuboid',vrep.simx_opmode_blocking);
[~, handles.leftFrame] = vrep.simxGetObjectHandle(handles.clientID,'Frame_Cuboid_Left',vrep.simx_opmode_blocking);
[~, handles.rightFrame] = vrep.simxGetObjectHandle(handles.clientID,'Frame_Cuboid_Right',vrep.simx_opmode_blocking);

handles.UR5 = [0 0]; % two UR5s
handles.UR5_base = [0 0];
handles.UR5_endeffector = [0 0];
handles.UR5_endeffector_frame = [0 0];
handles.UR5_joint = zeros(2,6); % six joints each;
handles.UR5_Tip_Handle = [];
handles.UR5_Force_Handle = [];


% First UR5
[~, handles.UR5(1)] = vrep.simxGetObjectHandle(handles.clientID,'world_visual',vrep.simx_opmode_blocking);

[~, handles.UR5_joint(1,1)] = vrep.simxGetObjectHandle(handles.clientID,'shoulder_pan_joint',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(1,2)] = vrep.simxGetObjectHandle(handles.clientID,'shoulder_lift_joint',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(1,3)] = vrep.simxGetObjectHandle(handles.clientID,'elbow_joint',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(1,4)] = vrep.simxGetObjectHandle(handles.clientID,'wrist_1_joint',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(1,5)] = vrep.simxGetObjectHandle(handles.clientID,'wrist_2_joint',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(1,6)] = vrep.simxGetObjectHandle(handles.clientID,'wrist_3_joint',vrep.simx_opmode_blocking);

[~, handles.UR5_base(1)] = vrep.simxGetObjectHandle(handles.clientID,'world_visual',vrep.simx_opmode_blocking);
[~, handles.UR5_endeffector(1)] = vrep.simxGetObjectHandle(handles.clientID,'UR1TipHandle',vrep.simx_opmode_blocking);
[~, handles.UR5_endeffector_frame(1)] = vrep.simxGetObjectHandle(handles.clientID,'Frame_UR5_1_Endeffector',vrep.simx_opmode_blocking);
[~, handles.UR5_Tip_Handle(1)] = vrep.simxGetObjectHandle(handles.clientID,'UR1TipHandle',vrep.simx_opmode_blocking);

[~, handles.UR5_Force_Handle(1)] = vrep.simxGetObjectHandle(handles.clientID,'ee_fixed_joint',vrep.simx_opmode_blocking);

% Second UR5
[~, handles.UR5(2)] = vrep.simxGetObjectHandle(handles.clientID,'world_visual0',vrep.simx_opmode_blocking);

[~, handles.UR5_joint(2,1)] = vrep.simxGetObjectHandle(handles.clientID,'shoulder_pan_joint0',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(2,2)] = vrep.simxGetObjectHandle(handles.clientID,'shoulder_lift_joint0',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(2,3)] = vrep.simxGetObjectHandle(handles.clientID,'elbow_joint0',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(2,4)] = vrep.simxGetObjectHandle(handles.clientID,'wrist_1_joint0',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(2,5)] = vrep.simxGetObjectHandle(handles.clientID,'wrist_2_joint0',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(2,6)] = vrep.simxGetObjectHandle(handles.clientID,'wrist_3_joint0',vrep.simx_opmode_blocking);

[~, handles.UR5_base(2)] = vrep.simxGetObjectHandle(handles.clientID,'world_visual0',vrep.simx_opmode_blocking);
[~, handles.UR5_endeffector(2)] = vrep.simxGetObjectHandle(handles.clientID,'UR2TipHandle',vrep.simx_opmode_blocking);
[~, handles.UR5_endeffector_frame(2)] = vrep.simxGetObjectHandle(handles.clientID,'Frame_UR5_2_Endeffector',vrep.simx_opmode_blocking);
[~, handles.UR5_Tip_Handle(2)] = vrep.simxGetObjectHandle(handles.clientID,'UR2TipHandle',vrep.simx_opmode_blocking);

[~, handles.UR5_Force_Handle(2)] = vrep.simxGetObjectHandle(handles.clientID,'ee_fixed_joint0',vrep.simx_opmode_blocking);

%startingJoints = [pi/2, pi/2, 0, pi/2, 0, 0];

% Simulation step(V-rep)
step = 10e-3;
trajectoryThreshold = 0.01;

UR5Pose = zeros(2,6);
UR5PoseSpeed = zeros(2,6);
UR5PosePrevious = zeros(2,6);
UR5JointArray = zeros(2,6);
UR5JointSpeed = zeros(2,6);
UR5PoseGlobal = zeros(2,3);

ur5_constant_matrices;

stepCount = 1;
trajectoryCount = 2;

forceControl = -50;

objectScript = 'Cuboid'; 
URJacobianCall = ["", ""];
URJacobianCall(1) = 'calculateJacobian_UR1';
URJacobianCall(2) = 'calculateJacobian_UR2';

%% Path generation

[~,cuboidPosition] = vrep.simxGetObjectPosition(handles.clientID,handles.cuboid,-1,vrep.simx_opmode_oneshot_wait);
[~,cuboidOrientation] = vrep.simxGetObjectOrientation(handles.clientID,handles.cuboid,-1,vrep.simx_opmode_oneshot_wait);

startPose = [double(cuboidPosition) double(cuboidOrientation)];

pathType = "circular";

radius = 0.25;
speed = 0.05;
cuboidWidth = 0.099;
trajectoryStart = 0;

circularPathDatabase = generate_object_path(startPose, pathType, step, radius, speed);
leftArmDatabase = generate_left_arm_database(circularPathDatabase,cuboidWidth);
rightArmDatabase = generate_right_arm_database(circularPathDatabase,cuboidWidth);

disp('Basic trajectories generated.');

%% Databases

UR5_1_JointSpeed = [];
UR5_2_JointSpeed = [];

UR5_1_JointPosition = [];
UR5_2_JointPosition = [];

UR5_1_Force = [];
UR5_2_Force = [];

ErroPosition_UR5_1 = [];
ErroPosition_UR5_2 = [];

Trajectory_UR5_1 = [];
Trajectory_UR5_2 = [];

ObjectPose = zeros(1,6);
ObjectTrajectory = [];
ErroObjectDatabase = [];

forceDatabase = [];

SimulationTime = [];

absErrorLeftDatabase = [];
absErrorLeftRotDatabase = [];
absErrorRightDatabase = [];
absErrorRightRotDatabase = [];

leftArmWdatabase = [];
leftArmLambdaDatabase = [];

errorCounter = 0;
errorLimit = 5000;
%% Simulation Initialization

vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);

[~,UR5JointArray(1,:)] = getUR5JointPositions(vrep,handles.clientID,handles.UR5_joint(1,:),vrep.simx_opmode_streaming);
[~,~] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_endeffector(1),handles.UR5_base(1),vrep.simx_opmode_streaming);
[~,~] = vrep.simxGetObjectOrientation(handles.clientID,handles.UR5_endeffector(1),handles.UR5_base(1),vrep.simx_opmode_streaming);
[~,UR5Pose(1,1:3)] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_base(1),-1,vrep.simx_opmode_streaming);
[~,UR5Pose(1,4:6)] = vrep.simxGetObjectOrientation(handles.clientID,handles.UR5_base(1),-1,vrep.simx_opmode_streaming);
[~,UR5PoseSpeed(1,1:3),UR5PoseSpeed(1,4:6)] = vrep.simxGetObjectVelocity(handles.clientID,handles.UR5_Tip_Handle(1),vrep.simx_opmode_streaming);
[~, forceStateUR51, UR5_1_Force(1:3), UR5_1_Force(4:6)] = vrep.simxReadForceSensor(handles.clientID,handles.UR5_Force_Handle(1),vrep.simx_opmode_streaming);
[~,UR5PoseGlobal(1,1:3)] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_Tip_Handle(1),-1,vrep.simx_opmode_streaming);


[~,UR5JointArray(2,:)] = getUR5JointPositions(vrep,handles.clientID,handles.UR5_joint(2,:),vrep.simx_opmode_streaming);
[~,~] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_endeffector(2),handles.UR5_base(2),vrep.simx_opmode_streaming);
[~,~] = vrep.simxGetObjectOrientation(handles.clientID,handles.UR5_endeffector(2),handles.UR5_base(2),vrep.simx_opmode_streaming);
[~,UR5Pose(2,1:3)] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_base(2),-1,vrep.simx_opmode_streaming);
[~,UR5Pose(2,4:6)] = vrep.simxGetObjectOrientation(handles.clientID,handles.UR5_base(2),-1,vrep.simx_opmode_streaming);
[~,UR5PoseSpeed(2,1:3),UR5PoseSpeed(2,4:6)] = vrep.simxGetObjectVelocity(handles.clientID,handles.UR5_Tip_Handle(2),vrep.simx_opmode_streaming);
[~, forceStateUR52, UR5_2_Force(1:3), UR5_2_Force(4:6)] = vrep.simxReadForceSensor(handles.clientID,handles.UR5_Force_Handle(2),vrep.simx_opmode_streaming);
[~,UR5PoseGlobal(2,1:3)] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_Tip_Handle(2),-1,vrep.simx_opmode_streaming);


[~,ObjectPose(1:3)] = vrep.simxGetObjectPosition(handles.clientID,handles.cuboid,-1,vrep.simx_opmode_streaming);
[~,ObjectPose(4:6)] = vrep.simxGetObjectOrientation(handles.clientID,handles.cuboid,-1,vrep.simx_opmode_streaming);


%% Simulation Start

vrep.simxSynchronousTrigger(clientID);

[~,UR5JointArray(1,:)] = getUR5JointPositions(vrep,handles.clientID,handles.UR5_joint(1,:),vrep.simx_opmode_buffer);
[~,~] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_endeffector(1),handles.UR5_base(1),vrep.simx_opmode_buffer);
[~,UR5Pose(1,1:3)] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_base(1),-1,vrep.simx_opmode_buffer);
[~,UR5Pose(1,4:6)] = vrep.simxGetObjectOrientation(handles.clientID,handles.UR5_base(1),-1,vrep.simx_opmode_buffer);
[~, forceStateUR51, UR5_1_Force(1:3), UR5_1_Force(4:6)] = vrep.simxReadForceSensor(handles.clientID,handles.UR5_Force_Handle(1),vrep.simx_opmode_buffer);
[~,UR5PoseSpeed(1,1:3),UR5PoseSpeed(1,4:6)] = vrep.simxGetObjectVelocity(handles.clientID,handles.UR5_Tip_Handle(1),vrep.simx_opmode_buffer);


[~,UR5JointArray(2,:)] = getUR5JointPositions(vrep,handles.clientID,handles.UR5_joint(2,:),vrep.simx_opmode_buffer);
[~,~] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_endeffector(2),handles.UR5_base(2),vrep.simx_opmode_buffer);
[~,UR5Pose(2,1:3)] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_base(2),-1,vrep.simx_opmode_buffer);
[~,UR5Pose(2,4:6)] = vrep.simxGetObjectOrientation(handles.clientID,handles.UR5_base(2),-1,vrep.simx_opmode_buffer);
[~, forceStateUR52, UR5_2_Force(1:3), UR5_2_Force(4:6)] = vrep.simxReadForceSensor(handles.clientID,handles.UR5_Force_Handle(2),vrep.simx_opmode_buffer);
[~,UR5PoseSpeed(2,1:3),UR5PoseSpeed(2,4:6)] = vrep.simxGetObjectVelocity(handles.clientID,handles.UR5_Tip_Handle(2),vrep.simx_opmode_buffer);


leftArmDatabaseUR5 = transform_trajectory_reference(leftArmDatabase, UR5Pose(1,:));
leftArmDatabaseUR5(:,4) = leftArmDatabaseUR5(:,4);
leftArmDatabaseUR5(:,5) = leftArmDatabaseUR5(:,5) - pi/2;
leftArmDatabaseUR5(:,6) = leftArmDatabaseUR5(:,6);

rightArmDatabaseUR5 = transform_trajectory_reference(rightArmDatabase, UR5Pose(2,:));
rightArmDatabaseUR5(:,4) = rightArmDatabaseUR5(:,4);
rightArmDatabaseUR5(:,5) = rightArmDatabaseUR5(:,5) + pi/2; 
rightArmDatabaseUR5(:,6) = rightArmDatabaseUR5(:,6) - pi;

[sizePath,~] = size(leftArmDatabaseUR5);

leftArmDatabaseSpeed = leftArmDatabaseUR5;
leftArmDatabaseSpeed(1,:) = zeros(1,6);

rightArmDatabaseSpeed = rightArmDatabaseUR5;
rightArmDatabaseSpeed(1,:) = zeros(1,6);

for i=2:sizePath
    leftArmDatabaseSpeed(i,:) = (leftArmDatabaseUR5(i,:) - leftArmDatabaseUR5(i-1,:))/step;
    rightArmDatabaseSpeed(i,:) = (rightArmDatabaseUR5(i,:) - rightArmDatabaseUR5(i-1,:))/step;
end
%
vrep.simxSetObjectOrientation(handles.clientID,handles.leftFrame,handles.UR5_base(1),leftArmDatabaseUR5(1,4:6),vrep.simx_opmode_oneshot);
vrep.simxSetObjectPosition(handles.clientID,handles.leftFrame,handles.UR5_base(1),leftArmDatabaseUR5(1,1:3),vrep.simx_opmode_oneshot);
vrep.simxSetObjectOrientation(handles.clientID,handles.rightFrame,handles.UR5_base(2),rightArmDatabaseUR5(1,4:6),vrep.simx_opmode_oneshot);
vrep.simxSetObjectPosition(handles.clientID,handles.rightFrame,handles.UR5_base(2),rightArmDatabaseUR5(1,1:3),vrep.simx_opmode_oneshot);
% 

vrep.simxSynchronousTrigger(clientID);

% 

[~,UR5Pose(1,1:3)] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_endeffector(1),handles.UR5_base(1),vrep.simx_opmode_buffer);
[~,UR5Pose(1,4:6)] = vrep.simxGetObjectOrientation(handles.clientID,handles.UR5_endeffector(1),handles.UR5_base(1),vrep.simx_opmode_buffer);

[~,UR5Pose(2,1:3)] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_endeffector(2),handles.UR5_base(2),vrep.simx_opmode_buffer);
[~,UR5Pose(2,4:6)] = vrep.simxGetObjectOrientation(handles.clientID,handles.UR5_endeffector(2),handles.UR5_base(2),vrep.simx_opmode_buffer);

%
vrep.simxSetObjectOrientation(handles.clientID,handles.UR5_endeffector_frame(1) ,handles.UR5_base(1),UR5Pose(1,4:6),vrep.simx_opmode_oneshot);
vrep.simxSetObjectPosition(handles.clientID,handles.UR5_endeffector_frame(1),handles.UR5_base(1),UR5Pose(1,1:3),vrep.simx_opmode_oneshot);
vrep.simxSetObjectOrientation(handles.clientID,handles.UR5_endeffector_frame(2) ,handles.UR5_base(2),UR5Pose(2,4:6),vrep.simx_opmode_oneshot);
vrep.simxSetObjectPosition(handles.clientID,handles.UR5_endeffector_frame(2),handles.UR5_base(2),UR5Pose(2,1:3),vrep.simx_opmode_oneshot);

vrep.simxSetObjectOrientation(handles.clientID,handles.UR5_Tip_Handle(1) ,handles.UR5_base(1),UR5Pose(1,4:6),vrep.simx_opmode_oneshot);
vrep.simxSetObjectPosition(handles.clientID,handles.UR5_Tip_Handle(1),handles.UR5_base(1),UR5Pose(1,1:3),vrep.simx_opmode_oneshot);
vrep.simxSetObjectOrientation(handles.clientID,handles.UR5_Tip_Handle(2) ,handles.UR5_base(2),UR5Pose(2,4:6),vrep.simx_opmode_oneshot);
vrep.simxSetObjectPosition(handles.clientID,handles.UR5_Tip_Handle(2),handles.UR5_base(2),UR5Pose(2,1:3),vrep.simx_opmode_oneshot);
%   

disp('Trajectory for left arm generated.');
disp('Trajectory for right arm generated.');

disp('Press any key to start simulation.');

pause;

[~,UR5Pose(1,1:3)] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_Tip_Handle(1),handles.UR5_base(1),vrep.simx_opmode_buffer);
[~,UR5Pose(1,4:6)] = vrep.simxGetObjectOrientation(handles.clientID,handles.UR5_Tip_Handle(1),handles.UR5_base(1),vrep.simx_opmode_buffer);
    
[~,UR5Pose(2,1:3)] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_Tip_Handle(2),handles.UR5_base(2),vrep.simx_opmode_buffer);
[~,UR5Pose(2,4:6)] = vrep.simxGetObjectOrientation(handles.clientID,handles.UR5_Tip_Handle(2),handles.UR5_base(2),vrep.simx_opmode_buffer);

UR5PosePrevious(1,:) = UR5Pose(1,:);
UR5PosePrevious(2,:) = UR5Pose(2,:);
[~,UR5PoseSpeed(1,1:3),UR5PoseSpeed(1,4:6)] = vrep.simxGetObjectVelocity(handles.clientID,handles.UR5_Tip_Handle(1),vrep.simx_opmode_buffer);
[~,UR5PoseSpeed(2,1:3),UR5PoseSpeed(2,4:6)] = vrep.simxGetObjectVelocity(handles.clientID,handles.UR5_Tip_Handle(2),vrep.simx_opmode_buffer);

[~,UR5PoseGlobal(1,1:3)] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_Tip_Handle(1),-1,vrep.simx_opmode_buffer);
[~,UR5PoseGlobal(2,1:3)] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_Tip_Handle(2),-1,vrep.simx_opmode_buffer);

erroPositionLeft = [0 0 0 0 0 0];
erroPositionRight = [0 0 0 0 0 0];
sumErrorPositionLeft = erroPositionLeft;
sumErrorPositionRight = erroPositionRight;

while trajectoryCount < length(leftArmDatabaseUR5)
    
    normalVector = (UR5Pose(1,1:3) - UR5Pose(2,1:3))/norm(UR5Pose(1,1:3) - UR5Pose(2,1:3));
    
    % Control Loop
    if (trajectoryStart == 1)
        [UR5JointSpeed(1,:),erroPositionLeft,w, lambda] = control_loop_PID(leftArmDatabaseUR5(trajectoryCount,:), UR5Pose(1,:), step, handles.clientID, objectScript, URJacobianCall(1), vrep, UR5PoseSpeed(1,:), 0, 0, 0, 0, 0, leftArmDatabaseUR5(trajectoryCount-1,:),erroPositionLeft, sumErrorPositionLeft, leftArmDatabaseSpeed(trajectoryCount,:));%forceControl,UR5_1_Force(4:6), speed);
    else
        [UR5JointSpeed(1,:),erroPositionLeft,w, lambda] = control_loop_PID(leftArmDatabaseUR5(trajectoryCount,:), UR5Pose(1,:), step, handles.clientID, objectScript, URJacobianCall(1), vrep, UR5PoseSpeed(1,:), 0, 0, 0, 0, 0, leftArmDatabaseUR5(trajectoryCount-1,:),erroPositionLeft, sumErrorPositionLeft, leftArmDatabaseSpeed(trajectoryCount,:));
    end
    %[UR5JointSpeed(2,:),erroPositionRight,~] = control_loop_PID(rightArmDatabaseUR5(trajectoryCount,:), UR5Pose(2,:), step, handles.clientID, objectScript, URJacobianCall(2), vrep, UR5PoseSpeed(2,:), 0, 0, 0, 0, 0, rightArmDatabaseUR5(trajectoryCount-1,:),erroPositionRight, sumErrorPositionRight, rightArmDatabaseSpeed(trajectoryCount,:));
    
    sumErrorPositionLeft = sumErrorPositionLeft + erroPositionLeft;
    sumErrorPositionRight = sumErrorPositionRight + erroPositionRight;
    
    vrep.simxPauseCommunication(handles.clientID, true);
    for i=1:6
        [~] = vrep.simxSetJointTargetVelocity(handles.clientID,handles.UR5_joint(1,i),UR5JointSpeed(1,i),vrep.simx_opmode_oneshot);
        [~] = vrep.simxSetJointTargetVelocity(handles.clientID,handles.UR5_joint(2,i),UR5JointSpeed(2,i),vrep.simx_opmode_oneshot);
    end
    vrep.simxPauseCommunication(handles.clientID, false);
    
    vrep.simxSynchronousTrigger(handles.clientID);
    
    % Update Frame Data
    
    vrep.simxSetObjectOrientation(handles.clientID,handles.leftFrame,handles.UR5_base(1),leftArmDatabaseUR5(trajectoryCount,4:6),vrep.simx_opmode_oneshot);
    vrep.simxSetObjectPosition(handles.clientID,handles.leftFrame,handles.UR5_base(1),leftArmDatabaseUR5(trajectoryCount,1:3),vrep.simx_opmode_oneshot);
    vrep.simxSetObjectOrientation(handles.clientID,handles.rightFrame,handles.UR5_base(2),rightArmDatabaseUR5(trajectoryCount,4:6),vrep.simx_opmode_oneshot);
    vrep.simxSetObjectPosition(handles.clientID,handles.rightFrame,handles.UR5_base(2),rightArmDatabaseUR5(trajectoryCount,1:3),vrep.simx_opmode_oneshot);
    
    
    % Update Pose Data
    [~, forceStateUR51, UR5_1_Force(1:3), UR5_1_Force(4:6)] = vrep.simxReadForceSensor(handles.clientID,handles.UR5_Force_Handle(1),vrep.simx_opmode_buffer);
    %UR5_1_Force
    
    [~,UR5PoseSpeed(1,1:3),UR5PoseSpeed(1,4:6)] = vrep.simxGetObjectVelocity(handles.clientID,handles.UR5_Tip_Handle(1),vrep.simx_opmode_buffer);
    [~,UR5PoseSpeed(2,1:3),UR5PoseSpeed(2,4:6)] = vrep.simxGetObjectVelocity(handles.clientID,handles.UR5_Tip_Handle(1),vrep.simx_opmode_buffer);
    
    [~,ObjectPose(1:3)] = vrep.simxGetObjectPosition(handles.clientID,handles.cuboid,-1,vrep.simx_opmode_buffer);
    [~,ObjectPose(4:6)] = vrep.simxGetObjectOrientation(handles.clientID,handles.cuboid,-1,vrep.simx_opmode_buffer);

    UR5PosePrevious(1,:) = UR5Pose(1,:);
    UR5PosePrevious(2,:) = UR5Pose(2,:);

    [~,UR5Pose(1,1:3)] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_Tip_Handle(1),handles.UR5_base(1),vrep.simx_opmode_buffer);
    [~,UR5Pose(1,4:6)] = vrep.simxGetObjectOrientation(handles.clientID,handles.UR5_Tip_Handle(1),handles.UR5_base(1),vrep.simx_opmode_buffer);
    
    [~,UR5Pose(2,1:3)] = vrep.simxGetObjectPosition(handles.clientID,handles.UR5_Tip_Handle(2),handles.UR5_base(2),vrep.simx_opmode_buffer);
    [~,UR5Pose(2,4:6)] = vrep.simxGetObjectOrientation(handles.clientID,handles.UR5_Tip_Handle(2),handles.UR5_base(2),vrep.simx_opmode_buffer);

    vrep.simxSetObjectOrientation(handles.clientID,handles.UR5_endeffector_frame(1) ,handles.UR5_base(1),UR5Pose(1,4:6),vrep.simx_opmode_oneshot);
    vrep.simxSetObjectPosition(handles.clientID,handles.UR5_endeffector_frame(1),handles.UR5_base(1),UR5Pose(1,1:3),vrep.simx_opmode_oneshot);
    vrep.simxSetObjectOrientation(handles.clientID,handles.UR5_endeffector_frame(2) ,handles.UR5_base(2),UR5Pose(2,4:6),vrep.simx_opmode_oneshot);
    vrep.simxSetObjectPosition(handles.clientID,handles.UR5_endeffector_frame(2),handles.UR5_base(2),UR5Pose(2,1:3),vrep.simx_opmode_oneshot);
    
    
    [~,UR5JointArray(1,:)] = getUR5JointPositions(vrep,handles.clientID,handles.UR5_joint(1,:),vrep.simx_opmode_streaming);
    [~,UR5JointArray(2,:)] = getUR5JointPositions(vrep,handles.clientID,handles.UR5_joint(2,:),vrep.simx_opmode_buffer);

    % Update Error Data
    
    % Position Error
    absErrorLeft = norm(erroPositionLeft(1:3));
    absErrorRight = norm(erroPositionRight(1:3));
    
    % Rotation Error
    R1 = eul2rotm(erroPositionLeft(4:6),'XYZ');
    absErrorLeftRot = abs(acos((trace(R1) - 1)/2));
    
    R2 = eul2rotm(erroPositionRight(4:6),'XYZ');
    absErrorRightRot = abs(acos((trace(R2) - 1)/2));
    
    errorLeft = [absErrorLeft absErrorLeftRot];
    errorRight = [absErrorRight absErrorRightRot];
    
    % Object Error
    
    matrixObject = transform_pose_to_SE3(ObjectPose);
    
    trajectoryMatrix = transform_pose_to_SE3(circularPathDatabase(trajectoryCount,:));
    trajectoryRotationMatrix = trajectoryMatrix(1:3,1:3);

    rotationErrorMatrix = matrixObject(1:3,1:3)'*trajectoryRotationMatrix;

    absErrorObject = norm(circularPathDatabase(trajectoryCount,1:3) - ObjectPose(1:3));
    absErrorObjectRotation = abs(acos((trace(rotationErrorMatrix) - 1)/2));
    
    errorObject = [absErrorObject absErrorObjectRotation];
    
   
    
    
    if (absErrorLeft < trajectoryThreshold)
        if (absErrorRight < trajectoryThreshold)
            if (trajectoryStart == 0)
                disp('Inicializando trajetoria')
                UR5JointArray(1,:)
                UR5JointArray(1,:)*180/pi
                pause();
                trajectoryStart = 1;
                stepCount = 1;
            end
            errorCounter = 0;
        else
            errorCounter = errorCounter + 1;
        end
    else
            errorCounter = errorCounter + 1;    
    end
    %pause(0.5);
    
    if (errorCounter >= errorLimit)
       disp('Path divergerd.')
       break
    end
    if (trajectoryStart == 1)
        trajectoryCount = trajectoryCount + 1;
        % Force Error
    
        forceDatabase(stepCount,:) = UR5_1_Force(3);
        %

        UR5_1_JointSpeed(stepCount,:) = UR5JointSpeed(1,:);
        UR5_2_JointSpeed(stepCount,:) = UR5JointSpeed(2,:);

        UR5_1_JointPosition(stepCount,:) = UR5JointArray(1,:);
        UR5_2_JointPosition(stepCount,:) = UR5JointArray(2,:);

        ErroPosition_UR5_1(stepCount,:) = erroPositionLeft;
        ErroPosition_UR5_2(stepCount,:) = erroPositionRight;

        Trajectory_UR5_1(stepCount,:) = UR5Pose(1,:);
        Trajectory_UR5_2(stepCount,:) = UR5Pose(2,:);

        Trajectory_UR5_1_desired(stepCount,:) = leftArmDatabaseUR5(trajectoryCount,:);
        Trajectory_UR5_2_desired(stepCount,:) = rightArmDatabaseUR5(trajectoryCount,:);

        absErrorLeftDatabase(stepCount) = absErrorLeft;
        absErrorLeftRotDatabase(stepCount) = absErrorLeftRot;
        absErrorRightDatabase(stepCount) = absErrorRight;
        absErrorRightRotDatabase(stepCount) = absErrorRightRot;
        
        leftArmWdatabase(stepCount) = w;
        leftArmLambdaDatabase(stepCount) = lambda;

        ObjectTrajectory(stepCount,:) = ObjectPose;
        ErroObjectDatabase(stepCount,:) = errorObject;
        
        SimulationTime(stepCount) = stepCount*step;
    
    end
    stepCount = stepCount + 1;
end

[simState] = vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);
%% Save Files

filename = '/home/nascimento/Projects/VREP/Scenes/Matlab-VREP/Figures/Dual_UR5/Kinematic/';
mkdir(filename);
filename = strcat(filename,'/S1K100-1-GI-NB.mat');
%save(filename,'SimulationTime','ErroObjectDatabase','ObjectTrajectory','absErrorLeftDatabase','absErrorRightDatabase','absErrorLeftRotDatabase','absErrorRightRotDatabase','UR5_1_JointSpeed','UR5_2_JointSpeed','UR5_1_JointPosition','UR5_2_JointPosition','ErroPosition_UR5_1','ErroPosition_UR5_2','Trajectory_UR5_1','Trajectory_UR5_2','Trajectory_UR5_1_desired','Trajectory_UR5_2_desired')
disp('Data Saved.');

%% Graphics

% figure(1)
% plot(SimulationTime,ErroObjectDatabase(:,1));
% title('Erro Absoluto do Objeto');
% grid on;
% xlabel('Tempo de Simulação');
% ylabel('Posição');
% 
% 
% figure(2)
% plot(SimulationTime,ErroObjectDatabase(:,2));
% title('Erro Absoluto do Objeto');
% grid on;
% xlabel('Tempo de Simulação');
% ylabel('Rotação');

% figure(3)
% plot(SimulationTime,ObjectTrajectory(:,1));
% title('Posição do Objeto');
% grid on;
% xlabel('Tempo de Simulação');
% ylabel('X');
% 
% 
% figure(4)
% plot(SimulationTime,ObjectTrajectory(:,2));
% title('Posição do Objeto');
% grid on;
% xlabel('Tempo de Simulação');
% ylabel('Y');
% 
% 
% figure(5)
% plot(SimulationTime,ObjectTrajectory(:,3));
% title('Posição do Objeto');
% grid on;
% xlabel('Tempo de Simulação');
% ylabel('Z');

figure(1)
plot(SimulationTime,absErrorLeftDatabase);
title('Erro Absoluto do UR51');
grid on;
xlabel('Tempo de Simulação');
ylabel('Posição');

figure(2)
plot(SimulationTime,absErrorLeftRotDatabase);
title('Erro Absoluto do UR51');
grid on;
xlabel('Tempo de Simulação');
ylabel('Rotação');

figure(3);
subplot(3,2,1);
plot(SimulationTime,UR5_1_JointSpeed(:,1));
title('Velocidade de junta - Braço 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

subplot(3,2,2);
plot(SimulationTime,UR5_1_JointSpeed(:,2));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q2)');

subplot(3,2,3);
plot(SimulationTime,UR5_1_JointSpeed(:,3));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q3)');

subplot(3,2,4);
plot(SimulationTime,UR5_1_JointSpeed(:,4));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q4)');

subplot(3,2,5);
plot(SimulationTime,UR5_1_JointSpeed(:,5));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q5)');

subplot(3,2,6);
plot(SimulationTime,UR5_1_JointSpeed(:,6));
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q6)');

figure(4);
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

figure(5);
subplot(3,2,1);
plot(SimulationTime,UR5_1_JointPosition(:,1)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q1');

subplot(3,2,2);
plot(SimulationTime,UR5_1_JointPosition(:,2)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q2');

subplot(3,2,3);
plot(SimulationTime,UR5_1_JointPosition(:,3)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q3');

subplot(3,2,4);
plot(SimulationTime,UR5_1_JointPosition(:,4)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q4');

subplot(3,2,5);
plot(SimulationTime,UR5_1_JointPosition(:,5)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q5');

subplot(3,2,6);
plot(SimulationTime,UR5_1_JointPosition(:,6)');
grid on;
xlabel('Tempo de Simulação');
ylabel('q6');

% 
% 
% figure(6)
% plot(SimulationTime,ObjectTrajectory(:,4));
% title('Rotação do Objeto');
% grid on;
% xlabel('Tempo de Simulação');
% ylabel('R(x)');
% 
% 
% figure(7)
% plot(SimulationTime,ObjectTrajectory(:,5));
% title('Rotação do Objeto');
% grid on;
% xlabel('Tempo de Simulação');
% ylabel('R(y)');
% 
% 
% figure(8)
% plot(SimulationTime,ObjectTrajectory(:,6));
% title('Rotação do Objeto');
% grid on;
% xlabel('Tempo de Simulação');
% ylabel('R(z)');

figure(9)
plot(SimulationTime,forceDatabase(:,1));
line([SimulationTime(1),SimulationTime(end)],[forceControl,forceControl]);
title('Força aplicada UR5 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('Força(N)');
%%
figure(10);
subplot(2,1,1);
plot(SimulationTime,leftArmWdatabase);
title('Proximidade de Singularidade');
grid on;
xlabel('Tempo de Simulação (s)');
ylabel('\omega');

subplot(2,1,2);
plot(SimulationTime,leftArmLambdaDatabase);
title('Fator de Amortecimento');
grid on;
xlabel('Tempo de Simulação (s)');
ylabel('\lambda');

figure(11);
subplot(3,1,1);
plot(SimulationTime,Trajectory_UR5_1(:,1)',SimulationTime,Trajectory_UR5_1_desired(:,1)', '--');
grid on;
xlabel('Tempo de Simulação (s)');
ylabel('X (m)');
legend('Trajetória realizada', 'Trajetória desejada');

subplot(3,1,2);
plot(SimulationTime,Trajectory_UR5_1(:,2)',SimulationTime,Trajectory_UR5_1_desired(:,2)', '--');
grid on;
xlabel('Tempo de Simulação (s)');
ylabel('Y (m)');
legend('Trajetória realizada', 'Trajetória desejada');


subplot(3,1,3);
plot(SimulationTime,Trajectory_UR5_1(:,3)',SimulationTime,Trajectory_UR5_1_desired(:,3)', '--');
grid on;
xlabel('Tempo de Simulação (s)');
ylabel('Z (m)');
legend('Trajetória realizada', 'Trajetória desejada');

fc = 3;
fs = 100;
[b,a] = butter(6,fc/(fs/2));
% 
% UR5_1_JointSpeedOut = UR5_1_JointSpeed;
% 
% UR5_1_JointSpeedOut(:,1) = filter(b,a,UR5_1_JointSpeed(:,1));
% UR5_1_JointSpeedOut(:,2) = filter(b,a,UR5_1_JointSpeed(:,2));
% UR5_1_JointSpeedOut(:,3) = filter(b,a,UR5_1_JointSpeed(:,3));
% UR5_1_JointSpeedOut(:,4) = filter(b,a,UR5_1_JointSpeed(:,4));
% UR5_1_JointSpeedOut(:,5) = filter(b,a,UR5_1_JointSpeed(:,5));
% UR5_1_JointSpeedOut(:,6) = filter(b,a,UR5_1_JointSpeed(:,6));

figure(12);
plot(SimulationTime,UR5_1_JointSpeedOut(:,1), SimulationTime,UR5_1_JointSpeedOut(:,2), SimulationTime,UR5_1_JointSpeedOut(:,3), SimulationTime,UR5_1_JointSpeedOut(:,4), SimulationTime,UR5_1_JointSpeedOut(:,5), SimulationTime,UR5_1_JointSpeedOut(:,6))
title('Velocidade de articulação')
grid on;
xlabel('Tempo de Simulação (s)');
ylabel('Velocidade (rad/s)');
l1 = legend('$\dot{q}_{1}$', '$\dot{q}_{2}$', '$\dot{q}_{3}$', '$\dot{q}_{4}$', '$\dot{q}_{5}$', '$\dot{q}_{6}$');
set(l1, 'Interpreter', 'latex');
%

filename = '/home/nascimento/Projects/VREP/Scenes/Matlab-VREP/Figures/Singularity';
mkdir(filename);
filename = strcat(filename,'/S1NODLS.mat');
%save(filename,'SimulationTime','UR5_1_JointSpeedOut','Trajectory_UR5_1','Trajectory_UR5_1_desired','leftArmLambdaDatabase','leftArmWdatabase')
disp('Data Saved.');

% figure(13)
% plot(SimulationTime,absErrorLeftDatabase);
% title('Erro Absoluto do UR5');
% grid on;
% xlabel('Tempo de Simulação (s)');
% ylabel('Posição (m)');
% 
% figure(14)
% plot(SimulationTime,absErrorLeftRotDatabase);
% title('Erro Absoluto do UR5');
% grid on;
% xlabel('Tempo de Simulação (s)');
% ylabel('Rotação (rad)');

fig = figure(10);
print(fig,'-dpng')
fig = figure(11);
print(fig,'-dpng')
fig = figure(12);
print(fig,'-dpng')
fig = figure(13);
print(fig,'-dpng')
fig = figure(14);
print(fig,'-dpng')

%plot_and_save;

%% Finalization

disp('Simulation ended');

vrep.simxFinish(handles.clientID);

vrep.delete();