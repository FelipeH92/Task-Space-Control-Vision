function jacob = quaternion_jacobian(angularVelocity, stepSize)
    jacob = zeros(4,3);
    quaternion = eul2quat(angularVelocity, 'XYZ');
    for i = 1:3
        angularTemp = angularVelocity;
        angularTemp(i) = angularTemp(i) + stepSize;
        quaternionTemp = eul2quat(angularTemp,'XYZ');
        jacob(:,i) = (quaternionTemp - quaternion)/stepSize;
    end
end