function Jacobian = get_Jacobian(clientID,object,functionCall,vrep)

    
    [~, ~, JacobianFloat, ~, ~] = vrep.simxCallScriptFunction(clientID,object,...
                                    vrep.sim_scripttype_childscript,char(functionCall),[],[],[],[],...
                                    vrep.simx_opmode_blocking);
    
    Jacobian = zeros(6,6);
    Jacobian(:,1) = JacobianFloat(1:6);
    Jacobian(:,2) = JacobianFloat(7:12);
    Jacobian(:,3) = JacobianFloat(13:18);
    Jacobian(:,4) = JacobianFloat(19:24);
    Jacobian(:,5) = JacobianFloat(25:30);
    Jacobian(:,6) = JacobianFloat(31:36);
    Jacobian = Jacobian';
    JacobClone = Jacobian;
    Jacobian(:,1) = JacobClone(:,6);
    Jacobian(:,2) = JacobClone(:,5);
    Jacobian(:,3) = JacobClone(:,4);
    Jacobian(:,4) = JacobClone(:,3);
    Jacobian(:,5) = JacobClone(:,2);
    Jacobian(:,6) = JacobClone(:,1);
%     JacobClone = Jacobian;
%     Jacobian(4,:) = JacobClone(6,:);
%     Jacobian(5,:) = JacobClone(4,:);
%     Jacobian(6,:) = JacobClone(5,:);

end