classdef ForwardMap < IMap
    %FORWARDMAP

    methods
        function obj = ForwardMap(nJoint, discretizeArray, options)
            arguments
                nJoint(1,1) double {mustBePositive}
                discretizeArray(:,:) double
                options.silent(1,1) logical = false
                options.jointMin(1,:) double = NaN
                options.jointMax(1,:) double = NaN
            end
            nJointArray = size(discretizeArray, 1);
            if nJointArray~=nJoint
                error('forwardMap:DimensionError', 'The nJoint does not match with dim 1 of discretizeArray');
            end

            if options.silent == false
                f = figure();
                a = axes(f);
                a.NextPlot = 'add';
                for iJoint = 1:size(discretizeArray,2)
                    plot(a, discretizeArray(:,iJoint))
                end
                if ~any(isnan(options.jointMin))
                    plot(a, options.jointMin, '--','Color','black','LineWidth',1.0)
                end
                if ~any(isnan(options.jointMax))
                    plot(a, options.jointMax, '--','Color','black','LineWidth',1.0)
                end
                xlabel(a,"Joint")
                ylabel(a,"Value")
                title(a,"Discretization Visualization")
                input('Displaying discretization. ''Enter'' to continue, ''Ctrl-C'' to interrupt ...','s');
            end

            obj.initialize(nJoint, discretizeArray);
        end
        
        function initialize(obj, nJoint, discretizeArray)
            obj.Domain = Rn(nJoint);
            [~, sizeIndex] = obj.Domain.activeDiscretize(discretizeArray);
            
            obj.Codomain = SE3();
            obj.Codomain.discretize(sizeIndex);
        end

        function compute(obj)
            model = UpperLimbModelToolbox();

            [dataQ, ~, nQ] = obj.Domain.getDataReshaped();
            [dataT, ~, nT] = obj.Codomain.getDataReshaped();
            if nQ ~= nT
                error("ForwardMap:internalError", "The size for data is not correctly initialized.")
            end
            
            progressbar = ProgressbarCollection();
            progressbar.setProgressMaximum(1, nQ, 'Running forward map');
            
            for iQ = 1:nQ
                dataT(:,:,iQ) = model.forwardKinematics(dataQ(:,iQ));
                progressbar.stepProgress(1)
            end
            progressbar.finishProgress(1)
            delete(progressbar)
            
            obj.Codomain.setDataReshaped(dataT)
        end
    end
end

