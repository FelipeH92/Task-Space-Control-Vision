%% Codigo de Testes

close all
clear all
clc

disp('Program started');
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
        disp('Connected to remote API server');

        % enable the synchronous mode on the client:
        vrep.simxSynchronous(clientID,true);

        % getting handles:
        [~, cuboidHandle] = vrep.simxGetObjectHandle(clientID,'Cuboid',vrep.simx_opmode_blocking);
        
        [~, UR5_1] = vrep.simxGetObjectHandle(clientID,'UR5_1',vrep.simx_opmode_blocking);
        UR5_1_joint_handles = [0 0 0 0 0 0];
        [returnCode, testaodamassa] = vrep.simxGetObjectHandle(clientID,'UR5_joint1',vrep.simx_opmode_blocking);
        [~, UR5_1_joint_handles(1)] = vrep.simxGetObjectHandle(clientID,'UR5_joint1',vrep.simx_opmode_blocking);
        [~, UR5_1_joint_handles(2)] = vrep.simxGetObjectHandle(clientID,'UR5_joint2',vrep.simx_opmode_blocking);
        [~, UR5_1_joint_handles(3)] = vrep.simxGetObjectHandle(clientID,'UR5_joint3',vrep.simx_opmode_blocking);
        [~, UR5_1_joint_handles(4)] = vrep.simxGetObjectHandle(clientID,'UR5_joint4',vrep.simx_opmode_blocking);
        [~, UR5_1_joint_handles(5)] = vrep.simxGetObjectHandle(clientID,'UR5_joint5',vrep.simx_opmode_blocking);
        [~, UR5_1_joint_handles(6)] = vrep.simxGetObjectHandle(clientID,'UR5_joint6',vrep.simx_opmode_blocking);
        
        [~, UR5_1_base_handle] = vrep.simxGetObjectHandle(clientID,'Frame_Base_1',vrep.simx_opmode_blocking);
        [~, UR5_1_endeffector_handle] = vrep.simxGetObjectHandle(clientID,'UR5_connection',vrep.simx_opmode_blocking);
        
        
        [~, UR5_2] = vrep.simxGetObjectHandle(clientID,'UR5_2',vrep.simx_opmode_blocking);
        
        
        % starting simulation:
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
        
        % getting poses
        % vrep.simxSynchronousTrigger(clientID);
        [~,cuboidPosition] = vrep.simxGetObjectPosition(clientID,cuboidHandle,-1,vrep.simx_opmode_streaming);
        [~,cuboidOrientation] = vrep.simxGetObjectOrientation(clientID,cuboidHandle,-1,vrep.simx_opmode_streaming);
        [~,UR51_joint_array] = getUR5JointPositions(vrep,clientID,UR5_1_joint_handles,vrep.simx_opmode_streaming);
        [~,UR51_pose_in_base] = vrep.simxGetObjectPosition(clientID,UR5_1_endeffector_handle,UR5_1_base_handle,vrep.simx_opmode_streaming);
        [~,UR51_world_position] = vrep.simxGetObjectPosition(clientID,UR5_1_base_handle,-1,vrep.simx_opmode_streaming);
        [~,UR51_world_orientation] = vrep.simxGetObjectOrientation(clientID,UR5_1_base_handle,-1,vrep.simx_opmode_streaming);
        
        vrep.simxSynchronousTrigger(clientID);
        
        [~,cuboidPosition] = vrep.simxGetObjectPosition(clientID,cuboidHandle,-1,vrep.simx_opmode_buffer);
        [~,cuboidOrientation] = vrep.simxGetObjectOrientation(clientID,cuboidHandle,-1,vrep.simx_opmode_buffer);
        [~,UR51_joint_array] = getUR5JointPositions(vrep,clientID,UR5_1_joint_handles,vrep.simx_opmode_buffer);
        [~,UR51_pose_in_base] = vrep.simxGetObjectPosition(clientID,UR5_1_endeffector_handle,UR5_1_base_handle,vrep.simx_opmode_buffer);
        [~,UR51_world_position] = vrep.simxGetObjectPosition(clientID,UR5_1_base_handle,-1,vrep.simx_opmode_buffer);
        [~,UR51_world_orientation] = vrep.simxGetObjectOrientation(clientID,UR5_1_base_handle,-1,vrep.simx_opmode_buffer);
        
        UR51_pose = double([UR51_world_position UR51_world_orientation]);
        
        q_test = UR51_joint_array;
        q_test(1) = q_test(1) - pi/2;
        q_test(2) = q_test(2) - pi/2;
        q_test(4) = q_test(4) - pi/2;
        disp('Cinematica direta');
        ur5_direct_kinematics(q_test);
        
        startPose = [double(cuboidPosition) double(cuboidOrientation)];
        pathType = "circular";

        step = 50e-3;
        radius = 0.2;
        speed = 0.1;
        width = 0.1;

        circularPathDatabase = generate_object_path(startPose, pathType, step, radius, speed);
        leftArmDatabase = generate_left_arm_database(circularPathDatabase,width);
        rightArmDatabase = generate_right_arm_database(circularPathDatabase,width);
        
        newLeftArmDatabase = transform_trajectory_reference(leftArmDatabase, UR51_pose);
        
        %plot3(circularPathDatabase(:,1),circularPathDatabase(:,2),circularPathDatabase(:,3),leftArmDatabase(:,1),leftArmDatabase(:,2),leftArmDatabase(:,3),rightArmDatabase(:,1),rightArmDatabase(:,2),rightArmDatabase(:,3))
        
        
        
        vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end

vrep.delete();

disp('Simulation ended');