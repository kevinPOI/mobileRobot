classdef robotTrajectory
    %ROBOTTRAJECTORY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        samplePoints
    end
    
    methods
        function obj = robotTrajectory(refControl)
            %ROBOTTRAJECTORY Construct an instance of this class
            %   Detailed explanation goes here
            sampleInterval = 0.05;

            obj.samplePoints=[];
            tf = refControl.getTrajectoryDuration();
            numberOfSample = floor(tf / sampleInterval);
            
            x0 = 0;
            y0 = 0;

            th = 0;
            x = x0;
            y = y0;
            disp("began intergration with number of sample");
            disp(numberOfSample);
            for i = 1:numberOfSample
                t = i * sampleInterval;
                [V, w] = refControl.computeControl(t);
                th = th + w/2 * sampleInterval;
                dx = V * cos(th) * sampleInterval;
                dy = V * sin(th) * sampleInterval;
                th = th + w/2 * sampleInterval;
                x = x + dx;
                y = y + dy;
                obj.samplePoints(i,:) = [t, x, y, th];
            end
            
            %scatter(obj.samplePoints(:,2), obj.samplePoints(:,3));


        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

