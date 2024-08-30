classdef AppViewUpperLimbRaw < handle
    %APPVIEWUPPERLIMBRAW 
    
    properties (SetAccess = private)
        % Application data model.
        Model(1, 1) AppModelUpperLimb
    
        % Listener object used to respond dynamically to model events.
        Listener(:, 1) event.listener {mustBeScalarOrEmpty}

        % the figure and axes of the view
        f(1,1)
        ax(1,1)

        % the toolbox model
        toolbox_model(1,1)
    end
    
    methods
        function obj = AppViewUpperLimbRaw(model, forwardMap)
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

            % attach the axes
            obj.f = figure();
            obj.ax = axes(obj.f);

            %initialize the model
            obj.toolbox_model = UpperLimbModelToolbox();

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
            T_EE = data.T_EE.value;
            %T_frame_cell = data.T_frame.value;

            cla(obj.ax);
            obj.ax.NextPlot = 'add';
            
            q = data.q.value;
            [TShoulder, TElbow, TWrist, TEE] = obj.toolbox_model.armPoseReferencePosition(q);
            TShoulder_sym = TShoulder;
            TShoulder_sym(3,4) = -TShoulder_sym(3,4);
            TCenter = eye(4);
            TCenter(2,4) = -0.2;

            T_rotation = eul2tform([pi/2, 0, 0], "XYZ");
            %T_rotation = eye(4);
            TShoulder = T_rotation * TShoulder;
            TShoulder_sym = T_rotation * TShoulder_sym;
            TCenter = T_rotation * TCenter;
            TElbow = T_rotation * TElbow;
            TWrist = T_rotation * TWrist;
            TEE = T_rotation * TEE;
            T_EE = T_rotation * T_EE;

            visualAbstractArm(obj.ax, TShoulder, TShoulder_sym, TCenter, TElbow, TWrist, TEE);

            visualFrame(obj.ax, T_EE, "scale", 0.15);

            axis equal
            grid on
            view(obj.ax,[75,23]);

            title(obj.ax,"Arm abstract visualization")
            xlabel(obj.ax,"x");
            ylabel(obj.ax,"y");
            zlabel(obj.ax,"z");
        end
    end
end

