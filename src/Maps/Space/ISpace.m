classdef ISpace < handle
    %ICODOMAIN 
    
    properties (SetAccess = protected)
        sizeData double {mustBePositive}

        sizeIndex double {mustBePositive}

        data double

        metadata struct = struct()
    end

    properties
        extraData struct = struct()
    end

    methods
        function [data, sizeData, sizeIndex] = getData(obj)
            data = obj.data;
            sizeData = obj.sizeData;
            sizeIndex = obj.sizeIndex;
        end

        function setData(obj, data)
            if ~isequal(size(data), size(obj.data))
                error('ISpace:invalidInputs',"the size of data does not match with requirement.")
            end
            obj.data = data;
        end

        function discretize(obj, sizeIndex)
            obj.sizeIndex = sizeIndex;
            obj.data = zeros([obj.sizeData, sizeIndex]);
        end
    end

    methods (Abstract)
        [data, sizeData, lengthIndex] = getDataReshaped(obj);
        
        setDataReshaped(obj, data);

    %     [index, value] = discretizeMatcher(obj, value)
    end
end

