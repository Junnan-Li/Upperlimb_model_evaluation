function jointValue = circularState(jointValue, maxPoints)
%CIRCULARSTATE normalize the jointValue value into a 2*pi interval.
%    jointValue: The inputed joint values.
%    maxPoints: the maximum point of the joint interval. the result would
%                   all between (maxPoints - 2pi, maxPoints]
    jointValue = mod(jointValue, 2*pi);
    
    jointValue(maxPoints <= 2*pi & maxPoints < jointValue) = ...
        jointValue(maxPoints < 2*pi & maxPoints < jointValue) - 2*pi;
    
    minPoints = maxPoints - 2*pi;
    jointValue(minPoints >= 0 & minPoints >= jointValue) = ...
        jointValue(minPoints >= 0 & minPoints >= jointValue) + 2*pi;
end

