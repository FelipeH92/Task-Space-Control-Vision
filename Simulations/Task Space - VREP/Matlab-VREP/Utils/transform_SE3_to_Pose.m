function pose = transform_SE3_to_Pose(SE3)
    angles = rotm2eul(SE3(1:3,1:3),'XYZ');
    pose = [SE3(1:3,4)' angles];
end