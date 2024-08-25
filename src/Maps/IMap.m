classdef IMap < handle
    %IMAP 
    
    properties (SetAccess = protected)
        Domain(1,1) ISpace = Rn(1)

        Codomain(1,1) ISpace = Rn(1)
    end

    methods
        initialize(obj)

        compute(obj)
    end
end

