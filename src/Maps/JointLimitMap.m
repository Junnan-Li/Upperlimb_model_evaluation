classdef JointLimitMap < IMetricMap
    %JOINTLIMITMAP 
    
    methods
        function obj = JointLimitMap(inputSpace)
            arguments
                inputSpace(1,1) Rn
            end
            obj@IMetricMap(inputSpace, 1)
        end

        function compute(obj)
            [dataQ, ~, nQ] = obj.Domain.getDataReshaped();
            [dataDist, ~, Dist] = obj.Codomain.getDataReshaped();
            if nQ ~= Dist
                error("ForwardMap:internalError", "The size for data is not correctly initialized.")
            end

            model = UpperLimbModelToolbox();
            par = 4*180/pi;      
            q_limits_low = model.joint_min;
            q_limits_high = model.joint_max;
            
            q = dataQ;
            Hdot = 1 / par * (q_limits_high - q_limits_low).^2 .* (2 * q - q_limits_high - q_limits_low) ./ ...
                                                    ((q_limits_high - q).^2 .* (q - q_limits_low).^2);
            
            metric_jl = 1./(sqrt(1+abs(Hdot)));
            dataDist = prod(metric_jl, 1);
            
            obj.Codomain.setDataReshaped(dataDist)
            obj.Codomain.extraData.name = "Joint Limit Penalty";
        end
    end
end

