function Trajectory = transform_trajectory_reference(trajectoryToTransform, transformToInvert)
    
    Trajectory = [];
    invertedTransform = inv(transform_pose_to_SE3(transformToInvert));
    
    for i = 1:length(trajectoryToTransform)
        transformTemp = transform_pose_to_SE3(trajectoryToTransform(i,:));
        newTransform = invertedTransform*transformTemp;
        thetaAngles = rotm2eul(newTransform(1:3,1:3),'XYZ');
        Trajectory(i,:) = [newTransform(1:3,4)' thetaAngles];
    end

end