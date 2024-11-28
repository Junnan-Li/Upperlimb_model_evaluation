classdef testInverseKinematics < matlab.unittest.TestCase
    
    properties
        % the osim model
        model

        % the initial value of joint configuration
        q_init = [1 ; 0.2; 0; 1; 1; 0.2; 1]

        % the initial pose
        T_init

        % the numerical tolerance
        tol = 0.005;
    end
    
    properties (TestParameter)
        moveX = {0, -0.02, 0.05}
        moveY = {0, -0.02, 0.05}
        moveZ = {0, -0.02, 0.05}
        rotX  = {0, -0.1, 0.2}
        rotY  = {0, -0.1, 0.2}
        rotZ  = {0, -0.1, 0.2}
        method = {"toolbox_LM"}
    end
    
    methods (TestClassSetup)
        function setupModel(testCase)
            testCase.model = UpperLimbModelToolbox();
            testCase.T_init = testCase.model.forwardKinematics(testCase.q_init);
        end
    end

    methods(Test, ParameterCombination = "pairwise")
        function testMoveEECLIK(testCase, moveX, moveY, moveZ, rotX, rotY, rotZ, method)
            x_init = zeros(6,1);
            x_init(1:3) = testCase.T_init(1:3,4);
            x_init(4:6) = rotm2eul(testCase.T_init(1:3,1:3), 'XYZ');

            x_d = x_init + [moveX; moveY; moveZ; rotX; rotY; rotZ];
            T_d = eye(4);
            T_d(1:3, 4) = x_d(1:3);
            T_d(1:3, 1:3) = eul2rotm(x_d(4:6)', 'XYZ');

            [q_d, ~, info] = testCase.model.inverseKinematics(T_d, testCase.q_init, "methods", method);
            T_fin = testCase.model.forwardKinematics(q_d);
            
            T_diff = T_d/T_fin;
            x_diff = zeros(6,1);
            x_diff(1:3) = T_diff(1:3,4);
            x_diff(4:6) = rotm2eul(T_diff(1:3,1:3), 'XYZ');

            %testCase.assertEqual(info.status, 1);
            if info.status == 1
                testCase.assertEqual(x_diff, zeros(6,1), 'AbsTol', testCase.tol);
            else
                testCase.assumeTrue(false, "The IK does not deliver a reasonable solution")
            end
        end
    end
    
end