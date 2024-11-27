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
        function obj = AppViewUpperLimbRaw(model)
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
            TR = data.convex_hull.value;
            TR_visible = data.convex_hull.visible;
            voxel_position = data.voxel_position.value;
            voxel_position_visible = data.voxel_position.visible;
            voxel_value = data.voxel_value.value;
            voxel_value_visible = data.voxel_value.visible;

            cla(obj.ax);
            obj.ax.NextPlot = 'add';
            
            % abstract arm
            [TShoulder, TElbow, TWrist, TEE] = obj.toolbox_model.armPoseReferencePosition(q);
            TShoulder_sym = TShoulder;
            TShoulder_sym(3,4) = -TShoulder_sym(3,4);
            TCenter = eye(4);
            TCenter(2,4) = -0.2;

            T_rotation = eul2tform([pi/2, 0, 0], "XYZ");
            TShoulder = T_rotation * TShoulder;
            TShoulder_sym = T_rotation * TShoulder_sym;
            TCenter = T_rotation * TCenter;
            TElbow = T_rotation * TElbow;
            TWrist = T_rotation * TWrist;
            TEE = T_rotation * TEE;
            T_EE = T_rotation * T_EE;

            visualAbstractArm(obj.ax, TShoulder, TShoulder_sym, TCenter, TElbow, TWrist, TEE);

            % EE frame
            visualFrame(obj.ax, T_EE, "scale", 0.15);

            % Triangularzation
            if TR_visible
                k = TR.ConnectivityList;
                Points = (T_rotation(1:3,1:3)* TR.Points')';
                x = Points(:,1);
                y = Points(:,2);
                z = Points(:,3);
                trimesh(k, x, y, z, 'EdgeColor', [0, 0, 0], 'EdgeAlpha', 0.3, 'FaceAlpha', 0);
            end

            % position
            if voxel_position_visible
                plot_position = T_rotation(1:3,1:3) * voxel_position;
                if voxel_value_visible
                    scatter3(obj.ax, plot_position(1,:), plot_position(2,:), plot_position(3,:), 30, voxel_value, 'filled');
                    colormap(obj.ax,"jet")
                    colorbar(obj.ax)
                else
                    scatter3(obj.ax, plot_position(1,:), plot_position(2,:), plot_position(3,:), 30, 'filled', 'o', 'MarkerFaceAlpha', 0.6, 'MarkerFaceColor', [153, 204, 255]/255, 'MarkerEdgeColor', [0,0,0])
                end
            end

            axis equal
            grid on
            %view(obj.ax,[75,23]);

            title(obj.ax,"Arm abstract visualization")
            xlabel(obj.ax,"x");
            ylabel(obj.ax,"y");
            zlabel(obj.ax,"z");
        end
    end
end

