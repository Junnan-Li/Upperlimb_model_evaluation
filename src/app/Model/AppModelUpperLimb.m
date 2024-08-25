classdef AppModelUpperLimb < handle
    %APPMODELUPPERLIMB The data model from upper limb app visualization
    
    properties ( SetAccess = private ) 
        % sturcture of progressbar data.
        %   q - the joint state value
        %   T_EE - the end effector frame
        %   T_frame - the other joint frames to be displayed
        Data(1,1) struct = struct('q', struct("value", zeros(7,1), "visible", true), ...
                                  'T_EE', struct("value", eye(4), "visible", true), ...
                                  'T_frame', struct("value", {{}}, "visible", true))
    end
    
    events ( NotifyAccess = private ) 
        % Event broadcast when the data is changed.
        DataChanged
    end

    methods
        function updateData(obj, q, T_EE, T_frame)
            % UPDATE update the data of joint state value, end effector
            % frame and other joint frames to be displayed.
            arguments
                obj 
                q (7,1) double
                T_EE (4,4) double
                T_frame (:,1) cell
            end
            obj.Data.q.value = q;
            obj.Data.T_EE.value = T_EE;
            obj.Data.T_frame.value = T_frame;
            notify(obj, "DataChanged")
        end

        function data = getData(obj)
            % GETDATA get the data structure
            data = obj.Data;
        end

        function setVisibility(obj, type, value)
            % SETVISIBILITY set the visibility of specific components
            arguments
                obj 
                type (:,1) char {mustBeMember(type,{'q','T_EE','T_frame'})}
                value (1,1) logical
            end
            obj.Data.(type).visible = value;
            notify(obj, "DataChanged")
        end

        function isVisible = getVisibility(obj, type)
            % GETVISIBILITY get the visibility of specific components
            arguments
                obj 
                type (:,1) char {mustBeMember(type,{'q','T_EE','T_frame'})}
            end
            isVisible = obj.Data.(type).visible;
        end
    end
end

