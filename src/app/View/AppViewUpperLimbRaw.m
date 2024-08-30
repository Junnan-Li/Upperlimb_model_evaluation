classdef AppViewUpperLimbRaw < handle
    %APPVIEWUPPERLIMBRAW 
    
    properties (SetAccess = private)
        % Application data model.
        Model(1, 1) AppModelUpperLimb
    
        % Listener object used to respond dynamically to model events.
        Listener(:, 1) event.listener {mustBeScalarOrEmpty}

        % the axes of the view
        ax(1,1)
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
            f = figure();
            obj.ax = axes(f);

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
            T_frame_cell = data.T_frame.value;

            cla(ax);
            
        end
    end
end

