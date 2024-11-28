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

    methods
        function [sizeData, sizeIndex] = activeDiscretize(obj, gridPosition, gridOrientation)
            arguments
                obj 
                gridPosition(:,3) double
                gridOrientation(4,4,:,:) double
            end
            
            nPosition = size(gridPosition,1);
            nDirection = size(gridOrientation, 3);
            nRotation = size(gridOrientation, 4);
            obj.sizeIndex = [nPosition, nDirection, nRotation];
            obj.discretize(obj.sizeIndex);

            obj.metadata.position = gridPosition;
            obj.metadata.orientation = gridOrientation;

            for iPosition=1:nPosition
                for iDirection=1:nDirection
                    for iRotation=1:nRotation
                        Tframe = gridOrientation(:,:,iDirection,iRotation);
                        Tframe(1:3,4) = Tframe(1:3,4) + gridPosition(iPosition,:)';
                        obj.data(:,:,iPosition,iDirection,iRotation) = Tframe;
                    end
                end
            end

            sizeData = obj.sizeData;
            sizeIndex = obj.sizeIndex;
        end
    end
end

