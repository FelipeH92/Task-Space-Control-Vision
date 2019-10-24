function circularPath = generate_circular_path(startPose, step, radius, speed)
    
    vectorNorm = speed*step; % Euclidian distance to increment each step.
    
    poseDatabase = [];
    dataCounter = 1;
    poseDatabase(dataCounter,:) = startPose;
    dataCounter = dataCounter + 1;
    
    distanceFromCenter = 0;
    distanceFromInitialPoint = 0;
    previousPoint = startPose;
    
    % Generate first radius distance in X direction.
    
    while (distanceFromInitialPoint < 0.1)
        % Calculates and increment database        
        nextPoint = previousPoint + [0 0 vectorNorm 0 0 0];
        poseDatabase(dataCounter,:) = nextPoint;
        
        % Update data
        dataCounter = dataCounter + 1;
        previousPoint = nextPoint;
        distanceFromInitialPoint = norm(nextPoint(1:3) - startPose(1:3));
    end
    
    startPose = nextPoint;
    
    while (distanceFromCenter <= radius)
        % Calculates and increment database
        nextPoint = previousPoint + [vectorNorm 0 0 0 0 0];
        poseDatabase(dataCounter,:) = nextPoint;
        
        % Update data
        dataCounter = dataCounter + 1;
        previousPoint = nextPoint;
        distanceFromCenter = norm(nextPoint(1:3) - startPose(1:3));
    end
    
    stepAngle= vectorNorm/distanceFromCenter;
    
    circumference = 2*pi*distanceFromCenter;
    traveledCircumference = 0;
    
    while (traveledCircumference < circumference)
        directionVector = previousPoint(1:3) - startPose(1:3);
        tangentVector = (directionVector)/norm(directionVector)*rotz(90);
        incrementPosition = vectorNorm*tangentVector*rotz(stepAngle);
        incrementPose = [incrementPosition 0 0 0];
        nextPoint = previousPoint + incrementPose;
        poseDatabase(dataCounter,:) = nextPoint;
        
        % Update data
        dataCounter = dataCounter + 1;
        previousPoint = nextPoint;
        traveledCircumference = traveledCircumference + vectorNorm;
    end
    
    circularPath = poseDatabase;
end