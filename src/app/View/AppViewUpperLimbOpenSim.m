classdef AppViewUpperLimbOpenSim < handle
    %APPVIEWUPPERLIMBOPENSIM the upper limb model with OpenSim visualization
    
    properties (SetAccess = private)
        % Application data model.
        Model(1, 1) AppModelUpperLimb
    
        % Listener object used to respond dynamically to model events.
        Listener(:, 1) event.listener {mustBeScalarOrEmpty}

        % the Opensim toolbox model
        opensim_model
        
        % the list of coordinate for visualization
        coord_list = {'elv_angle', 'shoulder_elv', 'shoulder_rot', 'elbow_flexion',...
                      'pro_sup', 'deviation', 'flexion'};
    end
    
    methods
        function obj = AppViewUpperLimbOpenSim(model)
            % PROGRESSBARGUI The constructor of the VIEWUPPERLIMBOPENSIM
            %   PROGRESSBARGUI(model, namedArgs) create a progressbar GUI view
            %
            %   model - the AppModelUpperLimb class for data
            %           notification.
            arguments
                model(1,1) AppModelUpperLimb
            end

            % attach the listener
            obj.Model = model; 
            obj.Listener = listener(obj.Model, ... 
                "DataChanged", @obj.onDataChanged); 

            % attach the folder
            root_folder = matlab.project.rootProject().RootFolder;
            model_path = fullfile(root_folder,"model","MOBL_ARMS_fixed_41.osim");

            % load the OpenSim model
            geometry_folder_path = fullfile(root_folder,"src","external","OpenSimToolbox","geometry_folder","Geometry_MoBL_ARMS");
            org.opensim.modeling.ModelVisualizer.addDirToGeometrySearchPaths(geometry_folder_path);
            obj.opensim_model = osim_model(model_path);

            % set the EE marker
            obj.opensim_model.delete_all_markers;
            body_name = {'hand'};           % name of the attached body
            pos_vec = {org.opensim.modeling.Vec3(0,-0.1,0)};     % relative position in corresponding body frame
            obj.opensim_model.add_marker_points('Hand_endeffector', body_name, pos_vec);
            
            % initialize the visualization window
            obj.opensim_model.set_visualize();
            obj.opensim_model.viz.setGroundHeight(-2);

            % Refresh the view for the first time
            obj.onDataChanged();
        end
    end

    methods (Access = protected)
        function onDataChanged(obj, ~, ~) 
            %ONDATACHANGED Listener callback, responding to the model event "DataChanged"
            obj.updateData(obj.Model);
        end

        function updateData(obj, model)
            % UPDATEDATA Update the view accroding the the data in model. 
            %   UPDATEDATA(obj, model) update the view accroding the the data in model
            data = model.getData();
            q = data.q.value;
            
            obj.opensim_model.set_coordinate_value(obj.coord_list,q);
            obj.opensim_model.model_visualize();
        end
    end
end

