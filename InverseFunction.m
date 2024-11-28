openProject("C:\Users\Intel NUC Extreme\Documents\Files\code\Upperlimb_model_evaluation\Upper_limb.prj")
load('C:\Users\Intel NUC Extreme\Documents\Files\code\Upperlimb_model_evaluation\data\forward_map_v1.mat')
inverseMap = InverseMap(forwardMap, 'silent',true);
[initialGuessList, matchCounter] = inverseMap.prefill(range);
updateCounter = inverseMap.iterate(initialGuessList,range);
save("data\inverse_map_v1_"+range(1)+"-"+range(2),"initialGuessList","inverseMap");
