function [returnCode,jointPositions] = getUR5JointPositions(remoteAPI,clientID, handleArray, operationMode)
    
    jointPositions = [];
    for i = 1:1:length(handleArray)
        [returnCode,jointValue] = remoteAPI.simxGetJointPosition(clientID,int32(handleArray(i)),operationMode);
        jointPositions = [jointPositions jointValue];
    end
    
end