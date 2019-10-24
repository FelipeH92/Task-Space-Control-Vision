function ObjectPath = generate_object_path(startPose, pathType, step, radius, speed)
    
    % Chose and generate specific path type.
    if (pathType == "circular") % Circular horizontal path
        ObjectPath = generate_circular_path(startPose, step, radius, speed);
    else
        msg = ['Path type ', pathType, ' is not available.'];
        disp(msg)
    end
end