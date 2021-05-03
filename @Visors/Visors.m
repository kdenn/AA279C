classdef Visors < handle
    properties
        
        % Initial conditions
        ICs % same as visorsStruct
        w0 
        q0
        
    end
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Constructor
        function obj = Visors(w0, q0)
            obj.ICs = visorsStruct();
            obj.w0 = w0;
            obj.q0 = q0;
        end
    end
end