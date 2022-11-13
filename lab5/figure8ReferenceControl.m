classdef figure8ReferenceControl
    %FIGURE8 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
      v
      sf
      tf
      kt
      kk
      ks
      kv
      beginTime
      tPause
      acc
      tramp
    end
    
    methods
        function obj = figure8ReferenceControl(Ks, Kv, tPause)
            %FIGURE8 Construct an instance of this class
            %   Detailed explanation goes here
            obj.v = 0.08;
            obj.sf = 1;
            obj.acc = 0.1;
            obj.tramp = obj.v / obj.acc;
            obj.tf = obj.sf / obj.v + obj.tramp;
            obj.kt = 2 * pi / obj.sf;
            obj.kk = 15.1084;
            obj.ks = Ks;
            obj.kv = Kv;
            obj.tf = (Ks/Kv)*obj.tf;
            obj.tPause = tPause;
            %obj.beginTime = robotIF.toc() + tPause;

        end
        
        function [V, w]= computeControl(obj,timeNow)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.beginTime = obj.tPause;
            T = timeNow - obj.beginTime;
            if T < 0%pre-pause
                V = 0;
                w = 0;
            elseif T < obj.tramp%ramp up
                t = (obj.kv / obj.ks) * T;
                V = obj.acc * T;
                
                s = V * t/2;
                K = (obj.kk / obj.ks) * sin(obj.kt * s);
                w = K * V
            elseif T < obj.tf - obj.tramp
                t = (obj.kv / obj.ks) * T;
                V = obj.v;
                
                s = obj.v * obj.tramp / 2 + V * (t - obj.tramp);
                K = (obj.kk / obj.ks) * sin(obj.kt * s);
                w = K * V;
            elseif T < obj.tf
                tconst = obj.tf - 2 * obj.tramp;
                t = (obj.kv / obj.ks) * T;
                V = obj.v - (t - obj.tramp - tconst) * obj.acc;
                s = obj.v * obj.tramp / 2 + obj.v * tconst + (obj.v * obj.tramp / 2 - V * obj.tramp / 2);
                K = (obj.kk / obj.ks) * sin(obj.kt * s);
                w = K * V;
            else
                V = 0;
                w = 0;
            end

            
        end
        
        function duration = getTrajectoryDuration(obj)
            duration = obj.tf + obj.tPause * 2;
        end
    end
end

