classdef UpperLimbModelToolbox < IUpperLimbModel
    %UPPERLIMBMODELOPENSIMTOOLBOX the Model for upper limb model, which 
    
    properties (SetAccess = private)
        % the osim_model class after initialization
        model

        % the relevant muscle
        muscle_list = {'ECRL','ECRB','ECU','FCR','FCU','EDCI'};

        % the relevant coordinate (joint)
        coord_list = {'elv_angle', 'shoulder_elv', 'shoulder_rot', 'elbow_flexion',...
                      'pro_sup', 'deviation', 'flexion'};

        % the lower limit of each joint
        joint_min = zeros(7,1)

        % the upper limit of each joint
        joint_max = zeros(7,1) 

        % the maximum point of the joint normalization
        joint_maxPoint = repelem(pi, 7, 1)

        % the temporary variable to eliminate the repetitive setting of
        % joint configuration
        q_last
    end
    
    methods
        % UpperLimbModelOpenSimToolbox constructor
        function obj = UpperLimbModelToolbox()
            import org.opensim.modeling.*

            % set the folder and import the model
            root_folder = matlab.project.rootProject().RootFolder;
            model_path = fullfile(root_folder,"model","MOBL_ARMS_fixed_41.osim");
            obj.model = osim_model(model_path);
            
            % attach the EE frame
            obj.model.delete_all_markers;
            body_name = {'hand'};           % name of the attached body
            pos_vec = {Vec3(0,-0.1,0)};     % relative position in corresponding body frame
            obj.model.add_marker_points('Hand_endeffector', body_name, pos_vec);

            % disable the clamp for the human arm
            for i = 1: length(obj.coord_list)
                obj.model.CoordinateSet.get(obj.coord_list{i}).set_clamped(false);
                obj.model.CoordinateSet.get(obj.coord_list{i}).setClamped(obj.model.state,false);
            end


            % update the joint limit
            for iJoint = 1:numel(obj.coord_list)
                coord = obj.model.CoordinateSet.get(obj.coord_list{iJoint});
                obj.joint_min(iJoint) = coord.get_range(0);
                obj.joint_max(iJoint) = coord.get_range(1);
            end
        end
    end

    methods
        function setConfiguration(obj, q)
            arguments
                obj 
                q(:,1) double {mustBeReal}
            end
            if ~isequal(obj.q_last, q)
                obj.model.set_coordinate_value(obj.coord_list,q);
                obj.q_last = q;
            end
        end

        function T = forwardKinematics(obj, q)
            obj.setConfiguration(q);
            [~, w_p, w_R] = obj.model.get_mp_frame(1);
            T = eye(4);
            T(1:3,1:3) = w_R;
            T(1:3,4) = w_p;
        end

        function jacobian = getGeometricJacobian(obj, q)
            obj.setConfiguration(q);
            jacobian = obj.model.getJacobian_mp_minimal(1);
        end

        function isValid = checkValidity(obj, q)
            isValid = false;
            if all(q <= obj.joint_max) && all(q >= obj.joint_min)
                isValid = true;
            end
        end
        
        function [q, isValid, info] = inverseKinematics(obj, T, qInit, options)
            arguments
                obj 
                T(4,4) double
                qInit
                options.methods(1,:) char {mustBeMember(options.methods,{'CLIK','toolbox_CLIK','toolbox_LM'})} = 'toolbox_LM' 
            end

            switch(options.methods)
                case "CLIK"
                    error("UpperLimbModelToolbox:internal", "Not implemented");
                case "toolbox_CLIK"
                    x_d = zeros(6,1);
                    x_d(1:3) = T(1:3,4);
                    x_d(4:6) = rotm2eul(T(1:3,1:3), 'XYZ');
                    obj.setConfiguration(qInit);
                    [q, info] =  obj.model.IK_numeric(obj.coord_list, 1, x_d);
                case "toolbox_LM"
                    x_d = zeros(6,1);
                    x_d(1:3) = T(1:3,4);
                    x_d(4:6) = rotm2eul(T(1:3,1:3), 'XYZ');
                    obj.setConfiguration(qInit);
                    [q, info] =  obj.model.IK_numeric_LM(obj.coord_list, 1, x_d);
                otherwise
                    error("UpperLimbModelToolbox:internal", "Should not come to this branch, internal error.")
            end

            q = circularState(q, obj.joint_maxPoint);
            isValid = obj.checkValidity(q);
        end

        function [TShoulder, TElbow, TWrist, TEE] = armPoseReferencePosition(obj, q)
            obj.setConfiguration(q);

            pShoulder = osimVec3ToArray(obj.model.BodySet.get('scapphant').getPositionInGround(obj.model.state));
            RShoulder = osimMatrix2matrix(obj.model.BodySet.get('scapphant').getRotationInGround(obj.model.state));
            TShoulder = eye(4);
            TShoulder(1:3, 4) = pShoulder;
            TShoulder(1:3, 1:3) = RShoulder;
            pElbow = osimVec3ToArray(obj.model.BodySet.get('ulna').getPositionInGround(obj.model.state));
            RElbow = osimMatrix2matrix(obj.model.BodySet.get('ulna').getRotationInGround(obj.model.state));
            TElbow = eye(4);
            TElbow(1:3, 4) = pElbow;
            TElbow(1:3, 1:3) = RElbow;
            pWrist = osimVec3ToArray(obj.model.BodySet.get('proximal_row').getPositionInGround(obj.model.state));
            RWrist = osimMatrix2matrix(obj.model.BodySet.get('proximal_row').getRotationInGround(obj.model.state));
            TWrist = eye(4);
            TWrist(1:3, 4) = pWrist;
            TWrist(1:3, 1:3) = RWrist;

            TEE = obj.forwardKinematics(q);
        end

        function [angle, posList] = nullspaceAngle(obj, q)
            obj.setConfiguration(q);
            [TShoulder, TElbow, ~, TEE] = obj.armPoseReferencePosition(q);
            pShoulder = TShoulder(1:3,4);
            pElbow = TElbow(1:3,4);
            pEE= TEE(1:3,4);

            posList = [pShoulder, pElbow, pEE];

            % common normal axis (shoulder-endEffetor)
            normal = pEE - pShoulder;
            normal = normal / norm(normal);

            if norm(normal - [0,0,1]) < 1e-5
                normReference = [1;0;0];
            else
                normReference = [0;0;1];
            end

            % reference plane vectors
            reference = cross(normReference,normal);
            reference = reference / norm(reference);

            % moving (targeted) plane vectors
            target = cross(pElbow - pShoulder, normal);
            if norm(target) <= 1e-4
                 warning("Output vector is numerically not fine for calculation [%s][%d]",mat2str(target),norm(target))
            end
            target = target/norm(target);

            % a robust way of calculating angles between two vectors
            crossRefTar = cross(reference,target);
            signedNormCrossRefTar = sign(dot(crossRefTar,normal)) * norm(crossRefTar);
            angle = atan2(signedNormCrossRefTar, dot(reference,target));
            
            % shift the angle, in order to have a [0, 2pi] output
            if angle < 0
                angle = angle + 2*pi;
            end
        end

        function [qArray, stateArray] = selfMotionManifold(obj, qInit, options)
            arguments
                obj 
                qInit
                options.sections(1,1) double {mustBePositive} = 24
                options.maxStep(1,1) double {mustBePositive} = 1000
                options.stepLength(1,1) double {mustBePositive} = 0.015
            end

            nR = numel(qInit);
            qArray = zeros(nR, options.sections);
            stateArray = repelem(-1, 1, options.sections);
            sectionLength = 2 * pi / options.sections;

            T_init = obj.forwardKinematics(qInit);
            J_init = obj.getGeometricJacobian(qInit);
            delta_q_last = null(J_init);
            
            %Tcell = cell(1, options.maxStep);
            %Qcell = cell(1, options.maxStep);
            q_s = qInit;
            for iNull = 1:options.maxStep
                %T_s = obj.forwardKinematics(q_s);   
                J_s = obj.getGeometricJacobian(q_s);
                %Tcell{iNull} = T_s;
                %Qcell{iNull} = q_s;

                % null uses SVD to identify the nullspace vectors
                nullvec = null(J_s);

                % the dot product of nullvec and last nullvec identify if
                % nullspace motion is on same direction.
                direction = dot(nullvec, delta_q_last);
                if direction < 0
                    nullvec = -nullvec;
                elseif direction < 0.1
                    warning('UpperLimbModelToolbox:numeric',"Direction vector close to orthogonal! " + direction);
                end
                
                % Use the q_value_temp to update
                q_value_temp = q_s + options.stepLength * nullvec;
                [q_value_temp, isValid, info_ik] = obj.inverseKinematics(T_init, q_value_temp);
                q_value_temp = circularState(q_value_temp, obj.joint_maxPoint);
                if info_ik.iter > 1
                    delta_q_last = circularDiff(q_value_temp, q_s) / norm(circularDiff(q_value_temp, q_s));
                else
                    delta_q_last = nullvec;
                end

                q_s = q_value_temp;
                
                [angle, ~] = obj.nullspaceAngle(q_s);
                section = floor(angle/sectionLength) + 1;
                if stateArray(section) == -1
                    qArray(:,section) = q_s;
                    stateArray(section) = isValid;
                end

                if all(stateArray~=-1)
                    break;
                end
            end
        end

        function [qArray, stateArray] = inverseKinematicsWithManifold(obj, T, qInit)
            arguments
                obj 
                T 
                qInit
            end

            nR = numel(qInit);
            qArray = zeros(nR, 24);
            stateArray = repelem(-1, 1, 24);

            [q, ~, info] = obj.inverseKinematics(T, qInit);
            if info.status == 0
                return
            end
            [qArray, stateArray] = selfMotionManifold(obj, q);
        end
    end
end

