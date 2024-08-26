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

        % CHECKVALIDITY check the validity of a single solution q 
        isValid = checkValidity(obj, q)

        % numeric inverse kinematics
        [q, isValid] = inverseKinematics(obj, T, qInit, options)

        % nullspace angle
        [angle, posList] = nullspaceAngle(obj, q)
        
        % self motion
        [qArray, stateArray] = selfMotionManifold(obj, qInit, options)
        
        % numeric inverse kinematics with all solutions
        [qArray, stateArray] = inverseKinematicsWithManifold(obj, T, qInit)
    end
end

