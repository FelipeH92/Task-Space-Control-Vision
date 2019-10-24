function [jointSpeed,error] = control_loop_jacobian_transpose(trajectoryStep,poseActual, stepSize, clientID, object, functionCall, vrep, endEffectorVelocity, normalVector, forceReading, forceControl,torqueVector, maxSpeed, previousTrajectory)% posePrevious)
    %% Error Calculation
    %poseActual
    %trajectoryStep
    forwardKinematics = transform_pose_to_SE3(poseActual);
    
    poseError = trajectoryStep(1:3) - poseActual(1:3);
    
    trajectoryMatrix = transform_pose_to_SE3(trajectoryStep);
    trajectoryRotationMatrix = trajectoryMatrix(1:3,1:3);
     
    rotationErrorMatrix = forwardKinematics(1:3,1:3)'*trajectoryRotationMatrix;
    %rotationErrorMatrix = trajectoryRotationMatrix*forwardKinematics(1:3,1:3)';
    
    rotationErrorEuler = rotm2eul(rotationErrorMatrix,'XYZ');
    
%     for i = 1:3
%         if rotationErrorEuler(i) > pi
%             rotationErrorEuler(i) = rotationErrorEuler(i) - 2*pi;
%         elseif rotationErrorEuler(i) < -pi
%            rotationErrorEuler(i) = rotationErrorEuler(i) + 2*pi;
%         end
%     end
    
    rotationErrorQuaternion = eul2quat(rotationErrorEuler,'XYZ');
    
    error = [poseError rotationErrorEuler];
    
    
    %% Jacobians
    
    Jacobian = get_Jacobian(clientID, object, functionCall, vrep);
    positionJacobian = Jacobian(1:3,:);
    rotationJacobianEuler = Jacobian(4:6,:);
    %[~,positionJacobian,rotationJacobianEuler] = jacobian(jointActual, stepSize);
    
    %rotationSpeed = (poseActual(4:6) - posePrevious(4:6))/stepSize;
    
    rotationJacobianQuaternion = quaternion_jacobian(endEffectorVelocity(4:6),stepSize);
    
    rotationJacobian = rotationJacobianQuaternion*rotationJacobianEuler;
    
    
    projectionMatrix =[1 0 sin(trajectoryStep(5));...
                           0 cos(trajectoryStep(4)) (-cos(trajectoryStep(5))*sin(trajectoryStep(4)));
                           0 sin(trajectoryStep(4)) (cos(trajectoryStep(4))*cos(trajectoryStep(5)))];
    
    
    jacobA = [positionJacobian; rotationJacobianEuler];
    w = sqrt(det(jacobA*jacobA'));
    
    %xdDesired = (trajectoryStep - previousTrajectory)/stepSize;
    
    %% Control Matrix
    %pseudo = pinv(error);
    %K = 10*(xdDesired - endEffectorVelocity)*pseudo
    K = 1*eye(6);
    
    alpha = dot(error',jacobA*jacobA'*error')/dot(jacobA*jacobA'*error',jacobA*jacobA'*error');
    
%     w0 = 0.04;
%     k0 = 0.01;
%     
%     if (w < w0)
%         lambda = k0*((1 - (w/w0))^2);
%     else
%         lambda = 0.0; % Damping factor
%     end
    
    %% Force Control
    if (forceControl ~= 0)
        maxForce = -200;
        if (forceReading < -200)
            forceReading = -200;
        end
        forceVector = rotz(-90)*rotx(-90)*(normalVector'*forceReading);
        forceDesired = rotz(-90)*rotx(-90)*(normalVector'*forceControl);
        errorForce = (forceDesired - forceVector);
        errorTorque = rotz(-90)*rotx(-90)*torqueVector';
        %errorForce = (errorForce'*normalVector')*normalVector;
        %errorTorque = (errorTorque'*normalVector')*normalVector;

        errorForce = [errorForce' [0 0 0]];
        %errorForce = errorForce*[normalVector 0 0 0]';
        
        
        Kf = maxSpeed/maxForce;
        Gain = 1;
        
        
        if (norm(errorForce(1:3)) < 0)
            if (norm(errorForce(1:3)) < (-20))
                Kp = 0;
            else
                Kp = (norm(errorForce(1:3))/20) + Gain;
            end
        else
           Kp = Gain;
        end
        print('teste');
        Kf = Kp*Kf*eye(6,6);
        
        
    end
    
    %% Control
    
    jointSpeedPositionControl = 2*alpha*eye(6)*(jacobA')*error';
    
    
    if (forceControl ~= 0)
        jacobG = projectionMatrix*jacobA(4:6,:);
        jacobG = [jacobA(1:3,:); jacobG(1:3,:)];
        jointSpeedForceControl = (jacobG'/(jacobG*jacobG'))*Kf*errorForce';
        jointSpeed = jointSpeedPositionControl + jointSpeedForceControl;
    else
        jointSpeed = jointSpeedPositionControl;
    end
    
end