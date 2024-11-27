classdef AppModelUpperLimb < handle
    %APPMODELUPPERLIMB The data model from upper limb app visualization
    
    properties ( SetAccess = private ) 
        % sturcture of progressbar data.
        %   q - the joint state value
        %   T_EE - the end effector frame
        %   T_frame - the other joint frames to be displayed
        Data(1,1) struct = struct('q', struct("value", [1.5708, 0.5236, 0, 0, 0, 0, 0], "visible", true), ...
                                  'T_EE', struct("value",  [0.8665   -0.4990   -0.0135    0.3258;
                                                            0.4984    0.8663   -0.0351   -0.5578;
                                                            0.0292    0.0237    0.9993    0.1899;
                                                                 0         0         0    1.0000], "visible", true), ...
                                  'T_frame', struct("value", {{}}, "visible", true),...
                                  'convex_hull', struct("value", [], "visible", false), ...
                                  'voxel_position',struct("value", [], "visible", false), ...
                                  'voxel_value',struct("value", [], "visible", false))
    end
    
    events ( NotifyAccess = private ) 
        % Event broadcast when the data is changed.
        DataChanged
    end

    methods
        function updateData(obj, type, value)
            % UPDATE update the data of joint state value, end effector
            % frame and other joint frames to be displayed.
            arguments
                obj 
                type (1,:) char {mustBeMember(type,{'q','T_EE','T_frame','convex_hull', 'voxel_position', 'voxel_value'})}
                value
            end
            obj.Data.(type).value = value;
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
                type (1,:) char {mustBeMember(type,{'q','T_EE','T_frame','convex_hull', 'voxel_position', 'voxel_value'})}
                value (1,1) logical
            end
            obj.Data.(type).visible = value;
            notify(obj, "DataChanged")
        end

        function isVisible = getVisibility(obj, type)
            % GETVISIBILITY get the visibility of specific components
            arguments
                obj 
                type (1,:) char {mustBeMember(type,{'q','T_EE','T_frame','convex_hull', 'voxel_position', 'voxel_value'})}
            end
            isVisible = obj.Data.(type).visible;
        end
    end
end

