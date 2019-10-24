function matrixSE3 = transform_pose_to_SE3(pose)
    matrixSE3 = eye(4);
    matrixSE3(1:3,1:3) = eul2rotm(pose(4:6),'XYZ');
    matrixSE3(1:3,4) = pose(1:3);

end