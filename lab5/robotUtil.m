classdef robotUtil
    %ROBOTMODEL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        wheel_dist = 0.09;
    end
    
    methods(Static = true)
        function [V, w] = vlvrToVw(vl, vr)
            V = (vl + vr) / 2;
            w = (vr - vl) / robotModel.W;
        end
        function [vl, vr] = VwTovlvr(V, w)
           vl = V - robotModel.W / 2 *w;
           vr = V + robotModel.W / 2 * w;
        end
    end
end

