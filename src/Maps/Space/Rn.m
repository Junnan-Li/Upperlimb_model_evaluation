classdef Rn < ISpace
    %RN 
    
    methods
        function obj = Rn(n)
            obj.sizeData = n;
            obj.metadata.Rn_n = n;
        end

        function [data, sizeData, lengthIndex] = getDataReshaped(obj)
            data = reshape(obj.data, obj.sizeData, []);
            sizeData = obj.sizeData;
            lengthIndex = size(data, 2);
        end

        function setDataReshaped(obj, data)
            nData = size(data, 2);
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
        function [sizeData, sizeIndex] = activeDiscretize(obj, discretizeArray)
            n_dimension = size(discretizeArray,1);
            n_discrete = size(discretizeArray,2);
            if n_dimension ~= obj.metadata.Rn_n
                error('ISpace:invalidInputs', "The first dimension of discretizeArray should match the dimension of euclidian space.")
            end
            
            sizeIndex = repelem(n_discrete, obj.metadata.Rn_n);
            obj.discretize(sizeIndex);
            
            obj.metadata.n_discrete = n_discrete;
            obj.metadata.discretizeArray = discretizeArray;
            
            data = discretizeArray(1,:);
            for iN = 2:obj.metadata.Rn_n
                dataExtend = repmat(data,1,n_discrete);
                dataAppend = repelem(discretizeArray(iN,:), 1, n_discrete^(iN-1));
                data = [dataExtend;dataAppend];
            end
            data = reshape(data,[n_dimension,sizeIndex]);
            assert(isequal(size(data),size(obj.data)));
            obj.data = data;
            
            sizeData = obj.sizeData;
            sizeIndex = obj.sizeIndex;
        end
    end
end

