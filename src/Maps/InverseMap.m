classdef InverseMap < IMap
    %INVERSEMAP 
   
    methods
        function obj = InverseMap(forwardMap, gridSize, nDirection, nRotation, nSection, nMaxFlip, options)
            arguments
                forwardMap(1,1) ForwardMap
                gridSize(1,1) double {mustBePositive} = 0.10
                nDirection(1,1) double {mustBePositive} = 8
                nRotation(1,1) double {mustBePositive} = 3
                nSection(1,1) double {mustBePositive} = 24
                nMaxFlip(1,1) double {mustBePositive} = 4
                options.silent(1,1) logical = false
            end
            
            % generate the convex hull and corresponding value
            T_data = forwardMap.Codomain.getDataReshaped();
            x = squeeze(T_data(1, 4, :));
            y = squeeze(T_data(2, 4, :));
            z = squeeze(T_data(3, 4, :));
            k = convhull(x, y, z, 'Simplify', true);
            TR = triangulation(k, x, y, z);
            TRCenter = incenter(TR);
            TRNormal = faceNormal(TR);

            % generate the grid position
            gridDistance = gridSize * 2;
            xDiscrete = [flip(0: -gridDistance: min(x)), gridDistance: gridDistance: max(x)];
            yDiscrete = [flip(0: -gridDistance: min(y)), gridDistance: gridDistance: max(y)];
            zDiscrete = [flip(0: -gridDistance: min(z)), gridDistance: gridDistance: max(z)];
            [xValue, yValue, zValue] = meshgrid(xDiscrete, yDiscrete, zDiscrete);
            
            % take the grid that is only inside the convex hull as gridPosition
            gridDiscrete = [xValue(:), yValue(:), zValue(:)];
            gridMask = logical(size(gridDiscrete, 1));
            for iPosition = 1:size(gridDiscrete,1)
                gridMask(iPosition) = inConvexhull(gridDiscrete(iPosition,:), TRCenter, TRNormal);
            end
            gridPosition = gridDiscrete(gridMask, :);

            % generate the grid orientation
            [xDir, yDir, zDir] = saffAndKuijlaars(gridSize, nDirection);
            tform = repmat(eye(4,4), 1, 1, nDirection, nRotation);
            for iDirection = 1:nDirection
                tform(1,4,iDirection,:) = xDir(iDirection);
                tform(2,4,iDirection,:) = yDir(iDirection);
                tform(3,4,iDirection,:) = zDir(iDirection);
                position = [xDir(iDirection); yDir(iDirection); zDir(iDirection)];

                % calculate the initial rotation matrix
                Rz = - position/norm(position);
                Rx = zeros(3,1);
                Rx(3) = norm([Rz(1),Rz(2)]);
                if abs(Rx(3)) < eps
                    Rx(1) = 1;
                    Rx(2) = 0;
                else
                    if Rz(3) < 0
                        Rx(1) = abs(Rz(3)) / Rx(3) * Rz(1);
                        Rx(2) = abs(Rz(3)) / Rx(3) * Rz(2);
                    else
                        Rx(1) = - abs(Rz(3)) / Rx(3) * Rz(1);
                        Rx(2) = - abs(Rz(3)) / Rx(3) * Rz(2);
                    end
                end
                Ry = cross(Rz, Rx);
                R = [Rx,Ry,Rz];

                rotationScalar = linspace(0, 2*pi, nRotation+1);
                for iRotation = 1:nRotation
                    iR = axang2rotm([Rz',rotationScalar(iRotation)])*R;
                    tform(1:3,1:3,iDirection,iRotation) = iR;
                end
            end
            gridOrientation = tform;

            % visualize and confirm
            model = UpperLimbModelToolbox();
            if options.silent == false
                f = figure();
                ax = axes(f);
                ax.NextPlot = 'add';

                q_init = [pi/2, 0.9750, -0.0542, 0.8215, -0.2708, 0, 0];

                [TShoulder, TElbow, TWrist, TEE] = model.armPoseReferencePosition(q_init);
                visualAbstractArm(ax, TShoulder, TElbow, TWrist, TEE)
                trimesh(k, x, y, z, 'EdgeColor', [0, 0, 0], 'EdgeAlpha', 0.2, 'FaceAlpha', 0);
                scatter3(ax, gridPosition(:,1), gridPosition(:,2), gridPosition(:,3), 10, 'filled', 'o', 'MarkerFaceAlpha', 0.6, 'MarkerFaceColor', [153, 204, 255]/255, 'MarkerEdgeColor', [0,0,0]);
                xlabel(ax,'x')
                ylabel(ax,'y')
                zlabel(ax,'z')
                axis equal
                title(ax,"Position Visualization")

                f = figure();
                ax2 = axes(f);
                ax2.NextPlot = 'add';
                for iDirection = 1:nDirection
                    for iRotation = 1:nRotation
                        visualFrame(ax2, gridOrientation(:, :, iDirection, iRotation),"scale", gridSize/2);
                    end
                end
                xlabel(ax2,'x')
                ylabel(ax2,'y')
                zlabel(ax2,'z')
                axis equal
                title(ax2,"Orientation Visualization")
                input('Displaying discretization. ''Enter'' to continue, ''Ctrl-C'' to interrupt ...','s');
            end

            nJoint = forwardMap.Domain.metadata.Rn_n;
            obj.initialize(nJoint, gridPosition, gridOrientation, forwardMap, nSection, nMaxFlip);
        end
        
        function initialize(obj, nJoint, gridPosition, gridOrientation, forwardMap, nSection, nMaxFlip)
            obj.Domain = SE3();
            [~, sizeIndex] = obj.Domain.activeDiscretize(gridPosition, gridOrientation);
            obj.Domain.extraData.forwardMap = forwardMap;
            
            sizeIndex = [sizeIndex, nSection, nMaxFlip];
            obj.Codomain = Rn(nJoint);
            obj.Codomain.discretize(sizeIndex);
            obj.Codomain.extraData.solutionStatus = -ones(sizeIndex);
        end

        function [matchCounter, updateCounter] = compute(obj)
            [initialGuessList, matchCounter] = obj.prefill();
            updateCounter = obj.iterate(initialGuessList);
        end
    end

    methods 
        function [initialGuessList, matchCounter] = prefill(obj, positionList, options)
            % propose initial guess based on the distance we know before.
            arguments
                obj 
                positionList(2,1) double
                options.positionWeight(1,1) double {mustBePositive} = 1
                options.rotationWeight(1,1) double {mustBePositive} = 0.03
            end
            [dataQForward, dataSizeQ, nQ] = obj.Domain.extraData.forwardMap.Domain.getDataReshaped();
            [dataTForward, ~, ~] = obj.Domain.extraData.forwardMap.Codomain.getDataReshaped();
            [dataTInverse, ~, sizeIndexInverse] = obj.Domain.getData();

            nPosition = sizeIndexInverse(1);
            nRotation = sizeIndexInverse(2);
            nDirection = sizeIndexInverse(3);
            initialGuessList = zeros(dataSizeQ, nPosition,nRotation,nDirection);

            % expand and calculate the difference between dataTForward and dataTInverse
            positionWeight = options.positionWeight;
            rotationWeight = options.rotationWeight;
            progressbar = ProgressbarCollection();
            progressbar.setProgressMaximum(1, positionList(2)-positionList(1)+1, 'Running inverse map matching');
            parStep = progressbar.parallelStepHandle();

            positionRange = positionList(1):positionList(2);
            for iPosition = positionRange
                TForward = repmat(dataTForward, 1, 1, 1, nRotation, nDirection);
                TInverse = permute(repmat(squeeze(dataTInverse(:,:,iPosition,:,:)), 1, 1, 1, 1, nQ),[1,2,5,3,4]);
                TInverse(1:3,1:3,:,:,:) =  permute(TInverse(1:3,1:3,:,:,:), [2,1,3,4,5]);
                TInverse(1:3,4,:,:,:) =  -pagemtimes(TInverse(1:3,1:3,:,:,:), TInverse(1:3,4,:,:,:));
                TDiff = pagemtimes(TInverse, TForward);

                % positional difference
                posdiff = squeeze(TDiff(1:3,4,:,:,:));
                % logarithmic differnce for rotation
                rotdiff = TDiff(1:3,1:3,:,:,:);
                theta = squeeze(acos((rotdiff(1,1,:,:,:)+rotdiff(2,2,:,:,:)+rotdiff(3,3,:,:,:)-1)/2));
                rotLog = zeros([3, size(theta)]);
                scale_factor = theta .* 1 ./(sin(theta)/2);
                scale_factor(theta==0) = 0;
                rotLog(1,:,:,:) = scale_factor .* squeeze(rotdiff(3,2,:,:,:) - rotdiff(2,3,:,:,:));
                rotLog(2,:,:,:) = scale_factor .* squeeze(rotdiff(1,3,:,:,:) - rotdiff(3,1,:,:,:));
                rotLog(3,:,:,:) = scale_factor .* squeeze(rotdiff(2,1,:,:,:) - rotdiff(1,2,:,:,:));

                posDist = sqrt(posdiff(1,:,:,:).^2 + posdiff(2,:,:,:).^2 + posdiff(3,:,:,:).^2);
                rotDist = sqrt(rotLog(1,:,:,:).^2 + rotLog(2,:,:,:).^2 + rotLog(3,:,:,:).^2);

                fullDist = squeeze(positionWeight*posDist + rotationWeight*rotDist);
                [~, I] = min(fullDist, [], 1);
                I = squeeze(I);
                dataQFWD = dataQForward;
                initialGuessList(:, iPosition, :, :) = reshape(dataQFWD(:, I), dataSizeQ, 1, nRotation, nDirection);

                send(parStep, 1);
            end
            progressbar.finishProgress(1);
            delete(progressbar);


            matchCounter = nQ;
        end

        function updateCounter = iterate(obj, initialGuessList, positionList)
            % ITERATE for a iteration based on the configuration of initialGuessList
            % if it is a new nullspace flip solution, save it 
            [dataT, ~, sizeIndexT] = obj.Domain.getData();
            [dataQ, ~, sizeIndexQ] = obj.Codomain.getData();
            solutionStatus = obj.Codomain.extraData.solutionStatus;
            if ~isequal(sizeIndexT,sizeIndexQ(1:3))
                error("ForwardMap:internalError", "The size for data is not correctly initialized.")
            end
            % if ~isequal(sizeIndexT, size(initialGuessList(2:4)))
            %     error("ForwardMap:internalError", "The size for initialGuessList is not correctly computed.")
            % end

            nPosition = sizeIndexT(1);
            nDirection = sizeIndexT(2);
            nRotation = sizeIndexT(3);
            nT = (positionList(2)-positionList(1)+1) * nDirection * nRotation;

            progressbar = ProgressbarCollection();
            progressbar.setProgressMaximum(1, nT, 'Running inverse map');
            parStep = progressbar.parallelStepHandle();

            %modelList(1:nDirection, 1:nRotation) = UpperLimbModelToolbox();
            model = UpperLimbModelToolbox();
            positionRange = positionList(1):positionList(2);
            for iPosition = positionRange
                for iDirection = 1:nDirection
                    for iRotation = 1:nRotation
                        %model = modelList(iDirection,iRotation);

                        T = dataT(:, :, iPosition, iDirection, iRotation);
                        qInit = initialGuessList(:, iPosition, iDirection, iRotation);
                        [qArray, stateArray] = model.inverseKinematicsWithManifold(T, qInit);

                        dataQ(:,iPosition,iDirection,iRotation,:,1) = qArray; 
                        solutionStatus(iPosition,iDirection,iRotation,:,1) = stateArray;

                        send(parStep, 1);
                    end
                end
            end
            progressbar.finishProgress(1);
            delete(progressbar);
            
            obj.Codomain.setData(dataQ);
            obj.Codomain.extraData.solutionStatus = solutionStatus;
            obj.Codomain.extraData.positionList = positionList;

            updateCounter = nT;
        end

        function sortFlip(obj)

        end
    end
end