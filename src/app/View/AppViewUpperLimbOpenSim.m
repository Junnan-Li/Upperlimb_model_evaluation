classdef AppViewUpperLimbOpenSim < handle
    %APPVIEWUPPERLIMBOPENSIM the upper limb model with OpenSim visualization
    
    properties (SetAccess = private)
        % Application data model.
        Model(1, 1) AppModelUpperLimb
    
        % Listener object used to respond dynamically to model events.
        Listener(:, 1) event.listener {mustBeScalarOrEmpty}

        % the Opensim model
        opensim_model

        % the Opensim model state
        state
        
        % the OpenSim coordinate
        CoordinateSet
        % the list of coordinate for visualization
        coord_list = {'elv_angle', 'shoulder_elv', 'shoulder_rot', 'elbow_flexion',...
                      'pro_sup', 'deviation', 'flexion'};

        % the Opensim visualizer
        viz
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
            obj.opensim_model = org.opensim.modeling.Model(model_path);

            % read the handles
            obj.CoordinateSet = obj.opensim_model.getCoordinateSet();
            MarkerSet = obj.opensim_model.getMarkerSet();
            BodySet = obj.opensim_model.getBodySet();

            % set the EE marker
            num_marker = MarkerSet.getSize;
            for i = 1:num_marker
                MarkerSet.remove(MarkerSet.get(0));
            end
            %MarkerSet = obj.opensim_model.getMarkerSet();
            
            % modify the model marker for display
            sphere_geometry = org.opensim.modeling.Sphere();
            sphere_geometry.set_radius(0.02);
            sphere_geometry.setColor(org.opensim.modeling.Vec3(1, 1,0));

            markeri = org.opensim.modeling.Body();
            markeri.setName('Hand_endeffector');
            markeri.setMass(0);
            markeri.setMassCenter(org.opensim.modeling.Vec3(0));
            markeri.setInertia(org.opensim.modeling.Inertia(0,0,0,0,0,0));

            jointi = org.opensim.modeling.WeldJoint("Weldjoint", ...
                                BodySet.get('hand'), ...
                                org.opensim.modeling.Vec3(0,-0.1,0), ...
                                org.opensim.modeling.Vec3(0), ...
                                markeri, ...
                                org.opensim.modeling.Vec3(0, 0, 0), ...
                                org.opensim.modeling.Vec3(0, 0, 0));

            obj.opensim_model.addBody(markeri);
            markeri.attachGeometry(sphere_geometry);
            obj.opensim_model.addJoint(jointi);
            
            % initialize the visualization window
            obj.opensim_model.setUseVisualizer(true);

            % initialize the state
            init_state = obj.opensim_model.initSystem();
            obj.state = init_state;

            % remove the clamped characteristics
            for i = 1: length(obj.coord_list)
                obj.CoordinateSet.get(obj.coord_list{i}).set_clamped(false);
                obj.CoordinateSet.get(obj.coord_list{i}).setClamped(obj.state,false);
            end

            %om.update_set(); % update all set and lists of the model
            obj.viz = obj.opensim_model.updVisualizer().updSimbodyVisualizer();
            obj.viz.setShowSimTime(false);
            obj.viz.setBackgroundColor(org.opensim.modeling.Vec3(0)); % white
            obj.viz.setGroundHeight(-1);
            
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
            obj.CoordinateSet = obj.opensim_model.getCoordinateSet();

            for i = 1: length(obj.coord_list)
                value_i = q(i);
                obj.CoordinateSet.get(obj.coord_list{i}).setValue(obj.state,value_i);
            end

            % update the visualization window
            obj.opensim_model.getVisualizer().show(obj.state);
        end
    end
end

