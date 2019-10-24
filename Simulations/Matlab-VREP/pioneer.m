vrep=remApi('remoteApi');
vrep.simxFinish(-1);

clientID=vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

if (clientID > -1)
    disp('Connected')
    
    %Handle
    [returnCode, leftMotor] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    [returnCode, ultraSonicSensor5] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking);
    [returnCode, camera] = vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking);

    %OtherCode
    returnCode=vrep.simxSetJointTargetVelocity(clientID,leftMotor,0.1,vrep.simx_opmode_blocking);
    
    [returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,ultraSonicSensor5,vrep.simx_opmode_streaming);
    [returnCode,reslution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_streaming);
    
    for i=1:50 
       [returnCode,detectionState,detectedPoint,~,~]=vrep.simxReadProximitySensor(clientID,ultraSonicSensor5,vrep.simx_opmode_buffer);
       [returnCode,reslution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_buffer);

       imshow(image)
       disp(norm(detectedPoint));
       pause(0.1); 
    end
    
    
    returnCode=vrep.simxSetJointTargetVelocity(clientID,leftMotor,0,vrep.simx_opmode_blocking);
    
    
    %code here
    
    vrep.simxFinish(-1);
end

vrep.delete();