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
        function T = forwardKinematics(obj, q)
            obj.model.set_coordinate_value(obj.coord_list,q);
            [~, w_p, w_R] = obj.model.get_mp_frame(1);
            T = zeros(4,4);
            T(1:3,1:3) = w_R;
            T(1:3,4) = w_p;
        end

        function jacobian = getGeometricJacobian(obj, q)
            obj.model.set_coordinate_value(obj.coord_list, q);
            jacobian = obj.model.getJacobian_mp_minimal(1);
        end
    end
end

