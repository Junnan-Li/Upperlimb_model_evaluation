classdef testInverseKinematics < matlab.unittest.TestCase
    
    properties
        % the osim model
        model

        % the initial value of joint configuration
        q_init = [1 ; 0.2; 0; 1; 1; 0.2; 1]

        % the initial pose
        T_init

        % the numerical tolerance
        tol = 0.001;
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

            testCase.assertEqual(info.status, 1);
            testCase.assertEqual(T_fin, T_d, 'AbsTol', testCase.tol);
        end
    end
    
end