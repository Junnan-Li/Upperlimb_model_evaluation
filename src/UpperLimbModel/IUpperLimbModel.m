classdef IUpperLimbModel < handle
    methods (Abstract)
        % FORWARDKINEMATICS Get the forward kinematics of the end-effector
        % frame in specific joint configuration
        % [x_p, w_p, w_R] = forwardKinematics(obj, q)
        % Input:
        %     q - the joint configuration
        % Output:
        %     T - the end-effector expression and frame transformation       
        T = forwardKinematics(obj, q)

        % GETGEOMETRICJACOBIAN the the jacobian of the end-effector frame in 
        % specific joint configuration
        % jacobian = getGeometricJacobian(obj, q)
        % Input:
        %     q - the joint configuration
        % Output:
        %     jacobian - the jacobian matrix of the robot.
        jacobian = getGeometricJacobian(obj, q)

        % CHECKVALIDITY check the validity of the 
        %isValid = checkValidity(obj, q)

        % numeric inverse kinematics
        

        % nullspace angle


        % nullspace motion step
        

        % do a full round of motion

    end
end

