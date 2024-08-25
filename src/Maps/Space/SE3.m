classdef SE3 < ISpace
    %SE3 

    methods
        function obj = SE3()
            obj.sizeData = [4, 4];
        end

        function [data, sizeData, lengthIndex] = getDataReshaped(obj)
            data = reshape(obj.data, 4, 4, []);
            sizeData = obj.sizeData;
            lengthIndex = size(data, 3);
        end

        function setDataReshaped(obj, data)
            nData = size(data, 3);
            nSizeIndex = 1;
            for iIndex = 1:numel(obj.sizeIndex)
                nSizeIndex = nSizeIndex * obj.sizeIndex(iIndex);
            end
            if nData ~= nSizeIndex
                error('ISpace:invalidInputs',"the size of data does not match with requirement.")
            end
            obj.data = reshape(data,[obj.sizeData, obj.sizeIndex]);
        end
    end
end

