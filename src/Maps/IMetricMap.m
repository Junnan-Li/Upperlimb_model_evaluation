classdef IMetricMap < IMap
    %METRICMAP 
    
    methods
        function obj = IMetricMap(inputSpace, outputDimension)
            arguments
                inputSpace(1,1) Rn
                outputDimension(1,1) double
            end
            obj.initialize(inputSpace, outputDimension)
        end

        function initialize(obj, inputSpace, outputDimension)
            obj.Domain = inputSpace;
            sizeIndex = obj.Domain.sizeIndex;
            
            obj.Codomain = Rn(outputDimension);
            obj.Codomain.discretize(sizeIndex);
        end
    end
end

