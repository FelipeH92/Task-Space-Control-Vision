function [jacob,positionJacobian,rotationJacobian] = jacobian(qMesured, stepSize)
    jacob = zeros(6);
    Transform = ur5_direct_kinematics(qMesured);
    TransformCartesian = [Transform(1:3,4)' rotm2eul(Transform(1:3,1:3),'XYZ')];
    for i = 1:6
        qTemp = qMesured;
        qTemp(i) = qTemp(i) + stepSize;
        cartesianTemp = ur5_direct_kinematics(qTemp);
        cartesianTemp = [cartesianTemp(1:3,4)' rotm2eul(cartesianTemp(1:3,1:3),'XYZ')];
        cartVecDif = cartesianTemp - TransformCartesian;
        if i > 3
            while cartVecDif(i) > pi
                cartVecDif(i) = cartVecDif(i) - 2*pi;
            end
            while cartVecDif(i) < -pi
                cartVecDif(i) = cartVecDif(i) + 2*pi;
            end
        end
        jacob(:,i) = cartVecDif / stepSize;
    end
    positionJacobian = jacob(1:3,:);
    rotationJacobian = jacob(4:6,:);
end