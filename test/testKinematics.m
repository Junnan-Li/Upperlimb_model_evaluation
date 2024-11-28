classdef testKinematics < matlab.unittest.TestCase
    
    properties
        % the osim model
        model

        % the initial value of joint configuration
        q_init = [0.3 ; 0.2; 0; 1; 1; 0; 1]

        % the initial position
        w_p

        % the initial rotation
        w_R

        % the initial Jacobian
        J

        % the numerical tolerance
        tol = 0.0001;
    end

    properties (TestParameter)
        moveX = {0, -0.001, 0.001}
        moveY = {0, -0.001, 0.001}
        moveZ = {0, -0.001, 0.001}
        rotX  = {0, -0.01, 0.01}
        rotY  = {0, -0.01, 0.01}
        rotZ  = {0, -0.01, 0.01}
    end

    methods (TestClassSetup)
        function setupModel(testCase)
            testCase.model = UpperLimbModelToolbox();
            T_frame = testCase.model.forwardKinematics(testCase.q_init);
            testCase.w_p = T_frame(1:3,4);
            testCase.w_R = T_frame(1:3,1:3);
            testCase.J = testCase.model.getGeometricJacobian(testCase.q_init);
        end
    end
    
    methods(Test, ParameterCombination = "pairwise")
        function testMoveEE(testCase, moveX, moveY, moveZ)
            x = [moveX; moveY; moveZ; 0; 0; 0];

            q_fin = testCase.q_init + pinv(testCase.J) * x;
            T_frame_fin = testCase.model.forwardKinematics(q_fin);
            w_p_fin = T_frame_fin(1:3, 4);

            testCase.assertEqual(w_p_fin, testCase.w_p + x(1:3),'AbsTol',testCase.tol);
        end

        function testRotEEX(testCase, rotX)
            x = [0; 0; 0; rotX; 0; 0];

            q_fin = testCase.q_init + pinv(testCase.J) * x;
            T_frame_fin = testCase.model.forwardKinematics(q_fin);
            w_R_fin = T_frame_fin(1:3,1:3);

            testCase.assertEqual(w_R_fin, eul2rotm(x(4:6)', "XYZ") * testCase.w_R, 'AbsTol',testCase.tol);
        end

        function testRotEEY(testCase, rotY)
            x = [0; 0; 0; 0; rotY; 0];

            q_fin = testCase.q_init + pinv(testCase.J) * x;
            T_frame_fin = testCase.model.forwardKinematics(q_fin);
            w_R_fin = T_frame_fin(1:3,1:3);

            testCase.assertEqual(w_R_fin, eul2rotm(x(4:6)', "XYZ") * testCase.w_R, 'AbsTol',testCase.tol);
        end

        function testRotEEZ(testCase, rotZ)
            x = [0; 0; 0; 0; 0; rotZ];

            q_fin = testCase.q_init + pinv(testCase.J) * x;
            T_frame_fin = testCase.model.forwardKinematics(q_fin);
            w_R_fin = T_frame_fin(1:3,1:3);

            testCase.assertEqual(w_R_fin, eul2rotm(x(4:6)', "XYZ") * testCase.w_R, 'AbsTol',testCase.tol);
        end
    end

    methods (Test)
        function testCheckValid(testCase)
            q_max = testCase.model.joint_max;
            q_min = testCase.model.joint_min;
            q_middle = (q_max+q_min)/2;
            q_lower = q_min - (q_max - q_min)/2;
            q_upper = q_max + (q_max - q_min)/2;
            
            testCase.assertTrue(testCase.model.checkValidity(q_max))
            testCase.assertTrue(testCase.model.checkValidity(q_min))
            testCase.assertTrue(testCase.model.checkValidity(q_middle))
            testCase.assertFalse(testCase.model.checkValidity(q_lower))
            testCase.assertFalse(testCase.model.checkValidity(q_upper))
        end
    end 
end