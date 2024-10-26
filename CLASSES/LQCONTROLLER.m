% Author: Martin Petré

% Date 12/19/2023

classdef LQCONTROLLER

    properties
        type (1,1) string
        gain double {mustBeReal, mustBeFinite}
    end
    
    methods
        function obj = LQCONTROLLER(type, gain)
            obj.type = type;
            obj.gain = gain;
        end
    end
end



