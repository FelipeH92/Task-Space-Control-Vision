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
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID == -1)
    disp('Failed connecting to remote API server');
    vrep.delete();
    return;
end

disp('Connected to remote API server');

handles = struct('clientID', clientID);

% enable the synchronous mode on the client:
vrep.simxSynchronous(handles.clientID,true);

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
[~, handles.UR5(1)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_1',vrep.simx_opmode_blocking);

[~, handles.UR5_joint(1,1)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_joint1',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(1,2)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_joint2',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(1,3)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_joint3',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(1,4)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_joint4',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(1,5)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_joint5',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(1,6)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_joint6',vrep.simx_opmode_blocking);

[~, handles.UR5_base(1)] = vrep.simxGetObjectHandle(handles.clientID,'Frame_Base_1',vrep.simx_opmode_blocking);
[~, handles.UR5_endeffector(1)] = vrep.simxGetObjectHandle(handles.clientID,'UR1TipHandle',vrep.simx_opmode_blocking);
[~, handles.UR5_endeffector_frame(1)] = vrep.simxGetObjectHandle(handles.clientID,'Frame_UR5_1_Endeffector',vrep.simx_opmode_blocking);
[~, handles.UR5_Tip_Handle(1)] = vrep.simxGetObjectHandle(handles.clientID,'UR1TipHandle',vrep.simx_opmode_blocking);

[~, handles.UR5_Force_Handle(1)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_connection',vrep.simx_opmode_blocking);

% Second UR5
[~, handles.UR5(2)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_2',vrep.simx_opmode_blocking);

[~, handles.UR5_joint(2,1)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_joint1#0',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(2,2)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_joint2#0',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(2,3)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_joint3#0',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(2,4)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_joint4#0',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(2,5)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_joint5#0',vrep.simx_opmode_blocking);
[~, handles.UR5_joint(2,6)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_joint6#0',vrep.simx_opmode_blocking);

[~, handles.UR5_base(2)] = vrep.simxGetObjectHandle(handles.clientID,'Frame_Base_2',vrep.simx_opmode_blocking);
[~, handles.UR5_endeffector(2)] = vrep.simxGetObjectHandle(handles.clientID,'UR2TipHandle',vrep.simx_opmode_blocking);
[~, handles.UR5_endeffector_frame(2)] = vrep.simxGetObjectHandle(handles.clientID,'Frame_UR5_2_Endeffector',vrep.simx_opmode_blocking);
[~, handles.UR5_Tip_Handle(2)] = vrep.simxGetObjectHandle(handles.clientID,'UR2TipHandle',vrep.simx_opmode_blocking);

[~, handles.UR5_Force_Handle(2)] = vrep.simxGetObjectHandle(handles.clientID,'UR5_connection#0',vrep.simx_opmode_blocking);

%startingJoints = [pi/2, pi/2, 0, pi/2, 0, 0];

% Simulation step(V-rep)
step = 10e-3;
trajectoryThreshold = 0.002;

UR5Pose = zeros(2,6);
UR5PoseSpeed = zeros(2,6);
UR5PoseSpeedPrevious = zeros(2,6);
UR5PoseAcceleration = zeros(2,6);
UR5PosePrevious = zeros(2,6);
UR5JointArray = zeros(2,6);
UR5JointSpeed = zeros(2,6);
UR5PoseGlobal = zeros(2,3);
UR5JointAcceleration = zeros(2,6);

previousJacobian1 = zeros(6,6);
previousJacobian2 = zeros(6,6);


ur5_constant_matrices;

stepCount = 1;
trajectoryCount = 2;

forceControl = -50;

objectScript = 'Cuboid'; 
URJacobianCall = ["", ""];
URJacobianCall(1) = 'calculateJacobian_UR1';
URJacobianCall(2) = 'calculateJacobian_UR2';

%% Filter Initialization

UR5SpeedFiltered_1 = zeros(1,6);
UR5SpeedFiltered_2 = zeros(1,6);

filterStep = 1;
filterFrequency = 3;

UR5_1_JointSpeed_Actual = [];
UR5_2_JointSpeed_Actual = [];

smooth_factor = 0.0457;%0.0687
smooth_factor_2 = 0.4543;%0.8626
smooth_factor_3 = 0.4543;%0.0687
smooth_factor_4 = 0.0457;

UR5SpeedFiltered_1_Previous_3 =  zeros(1,6);
UR5SpeedFiltered_2_Previous_3 =  zeros(1,6);

UR5SpeedFiltered_1_Previous_2 =  zeros(1,6);
UR5SpeedFiltered_2_Previous_2 =  zeros(1,6);

UR5SpeedFiltered_1_Previous =  zeros(1,6);
UR5SpeedFiltered_2_Previous =  zeros(1,6);

%% Path generation

[~,cuboidPosition] = vrep.simxGetObjectPosition(handles.clientID,handles.cuboid,-1,vrep.simx_opmode_oneshot_wait);
[~,cuboidOrientation] = vrep.simxGetObjectOrientation(handles.clientID,handles.cuboid,-1,vrep.simx_opmode_oneshot_wait);

startPose = [double(cuboidPosition) double(cuboidOrientation)];

pathType = "circular";

radius = 0.2;
speed = 0.05;
cuboidWidth = 0.101;
trajectoryStart = 0;

circularPathDatabase = generate_object_path(startPose, pathType, step, radius, speed);
leftArmDatabase = generate_left_arm_database(circularPathDatabase,cuboidWidth);
rightArmDatabase = generate_right_arm_database(circularPathDatabase,cuboidWidth);

disp('Basic trajectories generated.');

%% Databases

UR5_1_JointSpeed = zeros(4000,6);
UR5_2_JointSpeed = zeros(4000,6);

UR5_1_Position = zeros(4000,6);
UR5_2_Position = zeros(4000,6);

UR5_1_Pose_Speed = zeros(4000,6);
UR5_2_Pose_Speed = zeros(4000,6);

UR5_1_Pose_Speed_Abs = zeros(4000);
UR5_2_Pose_Speed_Abs = zeros(4000);

UR5_1_Pose_Acceleration = zeros(4000,6);
UR5_2_Pose_Acceleration = zeros(4000,6);

UR5_1_Pose_Acceleration_Abs = zeros(4000);
UR5_2_Pose_Acceleration_Abs = zeros(4000);

UR5_1_Force = zeros(4000,6);
UR5_2_Force = zeros(4000,6);

UR5_1_Joint_Acceleration = zeros(4000,6);
UR5_2_Joint_Acceleration = zeros(4000,6);

ErroPosition_UR5_1 = zeros(4000,6);
ErroPosition_UR5_2 = zeros(4000,6);

w1Database = zeros(4000);
w2Database = zeros(4000);

Trajectory_UR5_1 = zeros(4000,6);
Trajectory_UR5_2 = zeros(4000,6);

ObjectPose = zeros(1,6);
ObjectTrajectory = zeros(4000,6);
ErroObjectDatabase = zeros(4000,2);

forceDatabase = zeros(4000,6);

SimulationTime = zeros(4000);

absErrorLeftDatabase = zeros(4000);
absErrorRightDatabase = zeros(4000);

Trajectory_UR5_1_desired = zeros(4000,6);
Trajectory_UR5_2_desired = zeros(4000,6);
Max_Accel_UR5_1 = zeros(4000);
Max_Accel_UR5_2 = zeros(4000);

errorCounter = 0;
errorLimit = 1000;
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
leftArmDatabaseUR5(:,4) = leftArmDatabaseUR5(:,4) + pi/2;
leftArmDatabaseUR5(:,6) = leftArmDatabaseUR5(:,6) - pi/2;

rightArmDatabaseUR5 = transform_trajectory_reference(rightArmDatabase, UR5Pose(2,:));
rightArmDatabaseUR5(:,6) = rightArmDatabaseUR5(:,6) + pi/2; 
rightArmDatabaseUR5(:,4) = rightArmDatabaseUR5(:,4) - pi/2;

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
    
while trajectoryCount < length(leftArmDatabaseUR5)
    
    normalVector = (UR5Pose(1,1:3) - UR5Pose(2,1:3))/norm(UR5Pose(1,1:3) - UR5Pose(2,1:3));
    
    % Control Loop
    if (trajectoryStart == 1)
        [UR5JointSpeed(1,:),UR5JointAcceleration(1,:), erroPositionLeft, w1, previousJacobian1] = control_loop_acceleration(leftArmDatabaseUR5(trajectoryCount,:),UR5Pose(1,:),step,handles.clientID, objectScript, URJacobianCall(1), vrep, UR5PoseSpeed(1,:), normalVector, 0, 0, 0, 0,leftArmDatabaseUR5(trajectoryCount-1,:), previousJacobian1, UR5JointSpeed(1,:));%forceControl,UR5_1_Force(4:6), speed);
    else
        [UR5JointSpeed(1,:),UR5JointAcceleration(1,:),erroPositionLeft, w1, previousJacobian1] = control_loop_acceleration(leftArmDatabaseUR5(trajectoryCount,:),UR5Pose(1,:),step,handles.clientID, objectScript, URJacobianCall(1), vrep, UR5PoseSpeed(1,:), normalVector, 0, 0, 0, 0,leftArmDatabaseUR5(trajectoryCount-1,:), previousJacobian1, UR5JointSpeed(1,:));
    end
    [UR5JointSpeed(2,:),UR5JointAcceleration(2,:),erroPositionRight, w2, previousJacobian2] = control_loop_acceleration(rightArmDatabaseUR5(trajectoryCount,:),UR5Pose(2,:),step,handles.clientID, objectScript, URJacobianCall(2), vrep, UR5PoseSpeed(2,:), 0, 0, 0,0,0,rightArmDatabaseUR5(trajectoryCount-1,:), previousJacobian2, UR5JointSpeed(2,:));
    
    % Filter
%     UR5SpeedFiltered_1 = smooth_factor*UR5JointSpeed(1,:) + (smooth_factor_2)*(UR5SpeedFiltered_1_Previous) + smooth_factor_3*UR5SpeedFiltered_1_Previous_2 + smooth_factor_4*UR5SpeedFiltered_1_Previous_3;%(UR5SpeedFiltered_1*(filterStep-1) + UR5JointSpeed(1,:))/filterStep;
%     UR5SpeedFiltered_2 = smooth_factor*UR5JointSpeed(2,:) + (smooth_factor_2)*(UR5SpeedFiltered_2_Previous) + smooth_factor_3*UR5SpeedFiltered_2_Previous_2 + smooth_factor_4*UR5SpeedFiltered_2_Previous_3;%(UR5SpeedFiltered_2*(filterStep-1) + UR5JointSpeed(2,:))/filterStep;
%     
%     UR5SpeedFiltered_1_Previous_3 = UR5SpeedFiltered_1_Previous_2;
%     UR5SpeedFiltered_2_Previous_3 = UR5SpeedFiltered_2_Previous_2;
%     
%     UR5SpeedFiltered_1_Previous_2 = UR5SpeedFiltered_1_Previous;
%     UR5SpeedFiltered_2_Previous_2 = UR5SpeedFiltered_2_Previous;
% 
%     UR5SpeedFiltered_1_Previous = UR5SpeedFiltered_1;
%     UR5SpeedFiltered_2_Previous = UR5SpeedFiltered_2;
% 
%     ActualUR5Speed_1 = UR5SpeedFiltered_1;
%     ActualUR5Speed_2 = UR5SpeedFiltered_2;
    %
    
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
    
    UR5PoseAcceleration = UR5PoseSpeed - UR5PoseSpeedPrevious;
    
    UR5PoseSpeedPrevious = UR5PoseSpeed;
    
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
    
    % Force Error
    
    [~,index] = max(UR5JointSpeed(1,:));
    Max_Accel_UR5_1(stepCount) = UR5JointAcceleration(1,index);
    
    [~,index] = max(UR5JointSpeed(2,:));
    Max_Accel_UR5_2(stepCount) = UR5JointAcceleration(2,index);
    
    forceDatabase(stepCount,:) = UR5_1_Force(3);
    %
    
    UR5_1_JointSpeed(stepCount,:) = UR5JointSpeed(1,:);
    UR5_2_JointSpeed(stepCount,:) = UR5JointSpeed(2,:);
    
    UR5_1_Joint_Acceleration(stepCount,:) = UR5JointAcceleration(1,:);
    UR5_2_Joint_Acceleration(stepCount,:) = UR5JointAcceleration(2,:);
    
%     UR5_1_JointSpeed_Actual(stepCount,:) = ActualUR5Speed_1;
%     UR5_2_JointSpeed_Actual(stepCount,:) = ActualUR5Speed_2;

    UR5_1_Pose_Speed(stepCount,:) = UR5PoseSpeed(1,:);
    UR5_2_Pose_Speed(stepCount,:) = UR5PoseSpeed(2,:);
    
    UR5_1_Pose_Speed_Abs(stepCount) = norm(UR5PoseSpeed(1,1:3));
    UR5_2_Pose_Speed_Abs(stepCount) = norm(UR5PoseSpeed(2,1:3));

    UR5_1_Pose_Acceleration(stepCount,:) = UR5PoseAcceleration(1,:);
    UR5_2_Pose_Acceleration(stepCount,:) = UR5PoseAcceleration(2,:);

    UR5_1_Pose_Acceleration_Abs(stepCount) = norm(UR5PoseAcceleration(1,1:3));
    UR5_2_Pose_Acceleration_Abs(stepCount) = norm(UR5PoseAcceleration(2,1:3));
    
    UR5_1_Position(stepCount,:) = UR5JointArray(1,:);
    UR5_2_Position(stepCount,:) = UR5JointArray(2,:);

    ErroPosition_UR5_1(stepCount,:) = erroPositionLeft;
    ErroPosition_UR5_2(stepCount,:) = erroPositionRight;
    
    w1Database(stepCount) = w1;
    w2Database(stepCount) = w2;

    Trajectory_UR5_1(stepCount,:) = UR5Pose(1,:);
    Trajectory_UR5_2(stepCount,:) = UR5Pose(2,:);
    
    Trajectory_UR5_1_desired(stepCount,:) = leftArmDatabaseUR5(trajectoryCount,:);
    Trajectory_UR5_2_desired(stepCount,:) = rightArmDatabaseUR5(trajectoryCount,:);

    absErrorLeftDatabase(stepCount) = absErrorLeft;
    absErrorRightDatabase(stepCount) = absErrorRight;
    
    ObjectTrajectory(stepCount,:) = ObjectPose;
    ErroObjectDatabase(stepCount,:) = errorObject;
    
    
    SimulationTime(stepCount) = stepCount*step;
    
    if (absErrorLeft < trajectoryThreshold)
        if (absErrorRight < trajectoryThreshold)
            if (trajectoryStart == 0)
                disp('Inicializando trajetoria')
                pause();
                trajectoryStart = 1;
            end
            errorCounter = 0;
        else
            errorCounter = errorCounter + 1;
        end
    else
            errorCounter = errorCounter + 1;    
    end
    %pause(0.5);
    
%      if (errorCounter >= errorLimit)
%          disp('Path divergerd.')
%          %break
%      end
     if (trajectoryStart == 1)
         trajectoryCount = trajectoryCount + 1;
     end
    stepCount = stepCount + 1;
end
stepCount = stepCount-1;

vrep.simxPauseCommunication(handles.clientID, true);

for i=1:6
        [~] = vrep.simxSetJointTargetVelocity(handles.clientID,handles.UR5_joint(1,i),0,vrep.simx_opmode_oneshot);
        [~] = vrep.simxSetJointTargetVelocity(handles.clientID,handles.UR5_joint(2,i),0,vrep.simx_opmode_oneshot);
end

vrep.simxPauseCommunication(handles.clientID, false);
    
vrep.simxSynchronousTrigger(handles.clientID);

%% Graphics

figure(1)
plot(SimulationTime(1:stepCount),ErroObjectDatabase(1:stepCount,1));
title('Erro Absoluto do Objeto');
grid on;
xlabel('Tempo de Simulação');
ylabel('Posição');


figure(2)
plot(SimulationTime(1:stepCount),ErroObjectDatabase(1:stepCount,2));
title('Erro Absoluto do Objeto');
grid on;
xlabel('Tempo de Simulação');
ylabel('Rotação');

figure(3)
plot(SimulationTime(1:stepCount),absErrorLeftDatabase(1:stepCount));
title('Erro Absoluto do Braço 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('Erro');

figure(4)
plot(SimulationTime(1:stepCount),absErrorRightDatabase(1:stepCount));
title('Erro Absoluto do Braço 2');
grid on;
xlabel('Tempo de Simulação');
ylabel('Erro');

figure(5)
plot(SimulationTime(1:stepCount),w1Database(1:stepCount));
title('Distância de singularidade - Braço 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('w');

figure(6)
plot(SimulationTime(1:stepCount),w2Database(1:stepCount));
title('Distância de singularidade - Braço 2');
grid on;
xlabel('Tempo de Simulação');
ylabel('w');

figure(7)
plot(SimulationTime(1:stepCount),UR5_1_JointSpeed(1:stepCount,1));
title('Velocidade de junta - Braço 1 - q1');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

%
%
figure(8)
plot(SimulationTime(1:stepCount),UR5_1_JointSpeed(1:stepCount,2));
title('Velocidade de junta - Braço 1 - q2');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

%
%
figure(9)
plot(SimulationTime(1:stepCount),UR5_1_JointSpeed(1:stepCount,3));
title('Velocidade de junta - Braço 1 - q3');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

%
%
figure(10)
plot(SimulationTime(1:stepCount),UR5_1_JointSpeed(1:stepCount,4));
title('Velocidade de junta - Braço 1 - q4');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

%
%
figure(11)
plot(SimulationTime(1:stepCount),UR5_1_JointSpeed(1:stepCount,5));
title('Velocidade de junta - Braço 1 - q5');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

%
%
figure(12)
plot(SimulationTime(1:stepCount),UR5_1_JointSpeed(1:stepCount,6));
title('Velocidade de junta - Braço 1 - q6');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(q1)');

%
%

figure(13)
plot(SimulationTime(1:stepCount),UR5_1_Joint_Acceleration(1:stepCount,1));
title('Aceleração de junta - Braço 1 - q1');
grid on;
xlabel('Tempo de Simulação');
ylabel('Vd(q1)');

%
%
figure(14)
plot(SimulationTime(1:stepCount),UR5_1_Joint_Acceleration(1:stepCount,2));
title('Aceleração de junta - Braço 1 - q2');
grid on;
xlabel('Tempo de Simulação');
ylabel('Vd(q2)');

%
%
figure(15)
plot(SimulationTime(1:stepCount),UR5_1_Joint_Acceleration(1:stepCount,3));
title('Aceleração de junta - Braço 1 - q3');
grid on;
xlabel('Tempo de Simulação');
ylabel('Vd(q3)');

%
%
figure(16)
plot(SimulationTime(1:stepCount),UR5_1_Joint_Acceleration(1:stepCount,4));
title('Aceleração de junta - Braço 1 - q4');
grid on;
xlabel('Tempo de Simulação');
ylabel('Vd(q4)');

%
%
figure(17)
plot(SimulationTime(1:stepCount),UR5_1_Joint_Acceleration(1:stepCount,5));
title('Aceleração de junta - Braço 1 - q5');
grid on;
xlabel('Tempo de Simulação');
ylabel('Vd(q5)');

%
%
figure(18)
plot(SimulationTime(1:stepCount),UR5_1_Joint_Acceleration(1:stepCount,6));
title('Aceleração de junta - Braço 1 - q6');
grid on;
xlabel('Tempo de Simulação');
ylabel('Vd(q6)');


%
%
figure(19)
plot(SimulationTime(1:stepCount),UR5_1_Pose_Speed_Abs(1:stepCount));
title('Velocidade de Pose - UR5 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(m/s)');

figure(20)
plot(SimulationTime(1:stepCount),UR5_2_Pose_Speed_Abs(1:stepCount));
title('Velocidade de Pose - UR5 2');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(m/s)');

figure(21)
plot(SimulationTime(1:stepCount),UR5_1_Pose_Acceleration_Abs(1:stepCount));
title('Aceleração de Pose - UR5 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(m/s²)');

figure(22)
plot(SimulationTime(1:stepCount),UR5_2_Pose_Acceleration_Abs(1:stepCount));
title('Aceleração de Pose - UR5 2');
grid on;
xlabel('Tempo de Simulação');
ylabel('V(m/s²)');
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

 figure(23)
 plot(SimulationTime(1:stepCount),forceDatabase(1:stepCount,1));
 line([SimulationTime(1),SimulationTime(end)],[forceControl,forceControl]);
 title('Força aplicada UR5 1');
 grid on;
 xlabel('Tempo de Simulação');
 ylabel('Força(N)');

%

figure(24)
plot(SimulationTime(1:stepCount),Max_Accel_UR5_1(1:stepCount));
title('Aceleração da Junta Dominante - UR5 1');
grid on;
xlabel('Tempo de Simulação');
ylabel('A(m/s²)');

figure(25)
plot(SimulationTime(1:stepCount),Max_Accel_UR5_2(1:stepCount));
title('Aceleração da Junta Dominante - UR5 2');
grid on;
xlabel('Tempo de Simulação');
ylabel('A(m/s²)');


%plot_and_save;

%% Finalization

disp('Simulation ended');

vrep.simxFinish(handles.clientID);

vrep.delete();