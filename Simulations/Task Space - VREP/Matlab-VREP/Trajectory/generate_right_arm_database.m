function rightArmDatabase = generate_right_arm_database(objectPathDatabase, width)

    [r, ~] = size(objectPathDatabase);
    
    vector = [0 -width/2 0]*rotx(objectPathDatabase(1,4)*180/pi)*roty(objectPathDatabase(1,5)*180/pi)*rotz(objectPathDatabase(1,6)*180/pi);
    rightArmDatabase = objectPathDatabase;
    
    for i = 1:r
        rightArmDatabase(i,:) = rightArmDatabase(i,:) + [vector 0 0 0];
    end
end