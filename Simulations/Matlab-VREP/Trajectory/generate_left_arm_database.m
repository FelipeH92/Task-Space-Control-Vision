function leftArmDatabase = generate_left_arm_database(objectPathDatabase, width)

    [r, ~] = size(objectPathDatabase);
    
    vector = [0 width/2 0]*rotx(objectPathDatabase(1,4)*180/pi)*roty(objectPathDatabase(1,5)*180/pi)*rotz(objectPathDatabase(1,6)*180/pi);
    leftArmDatabase = objectPathDatabase;
    
    for i = 1:r
        leftArmDatabase(i,:) = leftArmDatabase(i,:) + [vector 0 0 0];
    end
end