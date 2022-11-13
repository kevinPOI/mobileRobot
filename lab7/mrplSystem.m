classdef mrplSystem < handle
    %MLSR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
        robIF
        ct
        x
        y
        th
    end
    
    methods
        function obj = mrplSystem()
            global encoderFrame;
            global encoderDataReady;
            global encoderDataTimeStamp;
            global encoderDataStarted;
            global encoderDataTimeStart;
            encoderDataStarted = false;
            obj.robIF = robotIF(1, true);
            obj.ct = controler(obj.robIF);
            obj.x = 0;
            obj.y = 0;
            obj.th = 0;
            %MLSR Construct an instance of this class
            %   Detailed explanation goes here
        end
        
        function output = runRelativeTrajactory(obj, v, x, y, th, sign)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            traj = cubicSpiralTrajectory.planTrajectory(x, y, th, sign);
            traj.planVelocities(v);
            n = traj.numSamples;
            traj.timeArray(n + 1) = traj.timeArray(n) + 1.5;
            traj.poseArray(:, n+1) = traj.poseArray(:,n);
            traj.VArray(n + 1) = traj.VArray(n);
            traj.wArray(n + 1) = traj.wArray(n);
            obj.ct = obj.ct.run(traj);

        end
        function obj = getOdoPosition(obj)
            disp("getting odo")
            obj.x = obj.x + obj.ct.odoPosition(1)*cos(obj.th) - obj.ct.odoPosition(2)*sin(obj.th);
            obj.y = obj.y + obj.ct.odoPosition(1)*sin(obj.th) + obj.ct.odoPosition(2)*cos(obj.th);
            obj.th = obj.th + obj.ct.odoPosition(3);

            disp("current odo readings are");
            disp(obj.x);
            disp(obj.y);
        end
        
        function convertRecord = plotRecord(obj, x0, y0, th0)%convert records to absolute and plot it
            record = obj.ct.records;
            tab = [cos(th0), -sin(th0), x0; 
                sin(th0), cos(th0), y0;
                0,0,1;];
            refRecords = [record(:,1),record(:,2),ones(length(record(:,1)),1)];
            newRefRecords = tab * transpose(refRecords);

            odoRecords = [record(:,4),record(:,5),ones(length(record(:,1)),1)];
            newOdoRecords = tab * transpose(odoRecords);
            %newRefRecords = transpose(refRecords);
            plot(newRefRecords(1,:), newRefRecords(2,:), '-k');
            plot(newOdoRecords(1,:), newOdoRecords(2,:),'-r');
        end
        function output = goToPoint(obj, v, x0, y0, th0, x1, y1, th1, sign)
            tba = [cos(th0), sin(th0), -cos(th0)*x0 - sin(th0) * y0; 
                -sin(th0), cos(th0), sin(th0)*x0-cos(th0)*y0;
                0,0,1;];
            t = [cos(th0), -sin(th0), x0; 
                sin(th0), cos(th0), y0;
                0,0,1;];
            disp("inverse transformation matrix is")
            disp(tba)
            result = tba * [x1; y1; 1];
            xr = result(1);
            yr = result(2);
            thr = th1 - th0;
            disp("executing relative trajectory:")
            disp([xr; yr; thr]);
            obj.runRelativeTrajactory(v, xr, yr, thr, sign);

        end
    end
end

