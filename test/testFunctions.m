classdef testFunctions < matlab.unittest.TestCase

    properties
        % the numerical tolerance
        tol = 1e-7;
    end
    
    methods(Test)
        function testCircularDiff(testCase)
            difference = circularDiff(2*pi, 0.001);
            testCase.assertEqual(difference, -0.001, 'AbsTol', testCase.tol);

            difference2 = circularDiff(0, 2*pi-0.002);
            testCase.assertEqual(difference2, 0.002, 'AbsTol', testCase.tol);

            difference3 = circularDiff(0.004, 0.001);
            testCase.assertEqual(difference3, 0.003, 'AbsTol', testCase.tol);

            difference4 = circularDiff([2*pi, 0, 0.004], [0.001, 2*pi-0.002, 0.001]);
            testCase.assertEqual(difference4, [-0.001,0.002,0.003], 'AbsTol', testCase.tol);
        end

        function testCircularState1(testCase)
            jointValue = circularState(0.1, pi);
            testCase.assertEqual(jointValue, 0.1, 'AbsTol', testCase.tol);

            jointValue = circularState(pi+0.1, pi);
            testCase.assertEqual(jointValue, -pi+0.1, 'AbsTol', testCase.tol);
            
            jointValue = circularState(pi, pi);
            testCase.assertEqual(jointValue, pi, 'AbsTol', testCase.tol);

            jointValue = circularState(-pi, pi);
            testCase.assertEqual(jointValue, pi, 'AbsTol', testCase.tol);
        end

        function testCircularState2(testCase)
            jointValue = circularState(0.1, 2*pi);
            testCase.assertEqual(jointValue, 0.1, 'AbsTol', testCase.tol);

            jointValue = circularState(-0.1, 2*pi);
            testCase.assertEqual(jointValue, 2*pi-0.1, 'AbsTol', testCase.tol);

            jointValue = circularState(2*pi, 2*pi);
            testCase.assertEqual(jointValue, 2*pi, 'AbsTol', testCase.tol);
            
            jointValue = circularState(pi, 0);
            testCase.assertEqual(jointValue, -pi, 'AbsTol', testCase.tol);

            jointValue = circularState(-pi, 0);
            testCase.assertEqual(jointValue, -pi, 'AbsTol', testCase.tol);

            jointValue = circularState(-2*pi, 0);
            testCase.assertEqual(jointValue, 0, 'AbsTol', testCase.tol);
        end
    end
    
end