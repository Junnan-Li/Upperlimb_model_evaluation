load('C:\Users\Intel NUC Extreme\Documents\Files\code\Upperlimb_model_evaluation\data\inverse_map_v1_1-15.mat')
nPosition = size(inverseMap.Codomain.extraData.solutionStatus,1);
nDirection = size(inverseMap.Codomain.extraData.solutionStatus,2);
nRotation = size(inverseMap.Codomain.extraData.solutionStatus,3);
nSection = size(inverseMap.Codomain.extraData.solutionStatus,4);
nFlip = size(inverseMap.Codomain.extraData.solutionStatus,5);

initialGuessListMerged = zeros(7,nPosition,nDirection,nRotation);
dataQ = zeros(7,nPosition,nDirection,nRotation,nSection,nFlip);
solutionStatus = zeros(nPosition,nDirection,nRotation,nSection,nFlip);

load('C:\Users\Intel NUC Extreme\Documents\Files\code\Upperlimb_model_evaluation\data\inverse_map_v1_1-15.mat')
rangeList=1:15;
initialGuessListMerged(:,rangeList,:,:) = initialGuessList(:,rangeList,:,:);
dataQ(:,rangeList,:,:,:,:) = inverseMap.Codomain.data(:,rangeList,:,:,:,:);
solutionStatus(rangeList,:,:,:,:) = inverseMap.Codomain.extraData.solutionStatus(rangeList,:,:,:,:);

load('C:\Users\Intel NUC Extreme\Documents\Files\code\Upperlimb_model_evaluation\data\inverse_map_v1_16-30.mat')
rangeList=16:30;
initialGuessListMerged(:,rangeList,:,:) = initialGuessList(:,rangeList,:,:);
dataQ(:,rangeList,:,:,:,:) = inverseMap.Codomain.data(:,rangeList,:,:,:,:);
solutionStatus(rangeList,:,:,:,:) = inverseMap.Codomain.extraData.solutionStatus(rangeList,:,:,:,:);


load('C:\Users\Intel NUC Extreme\Documents\Files\code\Upperlimb_model_evaluation\data\inverse_map_v1_31-45.mat')
rangeList=31:45;
initialGuessListMerged(:,rangeList,:,:) = initialGuessList(:,rangeList,:,:);
dataQ(:,rangeList,:,:,:,:) = inverseMap.Codomain.data(:,rangeList,:,:,:,:);
solutionStatus(rangeList,:,:,:,:) = inverseMap.Codomain.extraData.solutionStatus(rangeList,:,:,:,:);


load('C:\Users\Intel NUC Extreme\Documents\Files\code\Upperlimb_model_evaluation\data\inverse_map_v1_46-60.mat')
rangeList=46:60;
initialGuessListMerged(:,rangeList,:,:) = initialGuessList(:,rangeList,:,:);
dataQ(:,rangeList,:,:,:,:) = inverseMap.Codomain.data(:,rangeList,:,:,:,:);
solutionStatus(rangeList,:,:,:,:) = inverseMap.Codomain.extraData.solutionStatus(rangeList,:,:,:,:);


load('C:\Users\Intel NUC Extreme\Documents\Files\code\Upperlimb_model_evaluation\data\inverse_map_v1_61-75.mat')
rangeList=61:75;
initialGuessListMerged(:,rangeList,:,:) = initialGuessList(:,rangeList,:,:);
dataQ(:,rangeList,:,:,:,:) = inverseMap.Codomain.data(:,rangeList,:,:,:,:);
solutionStatus(rangeList,:,:,:,:) = inverseMap.Codomain.extraData.solutionStatus(rangeList,:,:,:,:);


load('C:\Users\Intel NUC Extreme\Documents\Files\code\Upperlimb_model_evaluation\data\inverse_map_v1_76-90.mat')
rangeList=76:90;
initialGuessListMerged(:,rangeList,:,:) = initialGuessList(:,rangeList,:,:);
dataQ(:,rangeList,:,:,:,:) = inverseMap.Codomain.data(:,rangeList,:,:,:,:);
solutionStatus(rangeList,:,:,:,:) = inverseMap.Codomain.extraData.solutionStatus(rangeList,:,:,:,:);


load('C:\Users\Intel NUC Extreme\Documents\Files\code\Upperlimb_model_evaluation\data\inverse_map_v1_91-105.mat')
rangeList=91:105;
initialGuessListMerged(:,rangeList,:,:) = initialGuessList(:,rangeList,:,:);
dataQ(:,rangeList,:,:,:,:) = inverseMap.Codomain.data(:,rangeList,:,:,:,:);
solutionStatus(rangeList,:,:,:,:) = inverseMap.Codomain.extraData.solutionStatus(rangeList,:,:,:,:);


load('C:\Users\Intel NUC Extreme\Documents\Files\code\Upperlimb_model_evaluation\data\inverse_map_v1_106-138.mat')
rangeList=106:138;
initialGuessListMerged(:,rangeList,:,:) = initialGuessList(:,rangeList,:,:);
dataQ(:,rangeList,:,:,:,:) = inverseMap.Codomain.data(:,rangeList,:,:,:,:);
solutionStatus(rangeList,:,:,:,:) = inverseMap.Codomain.extraData.solutionStatus(rangeList,:,:,:,:);

