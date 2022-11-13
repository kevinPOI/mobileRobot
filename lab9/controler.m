

classdef controler < handle
    %CONTROLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robIF
        odoPosition
        records
        vArray
    end
    
    methods
        function obj = controler(robIF)
            global encoderFrame;
            global encoderDataReady;
            global encoderDataTimeStamp;
            global encoderDataStarted;
            global encoderDataTimeStart;
            
            obj.robIF = robIF;
            obj.robIF.encoders.NewMessageFcn = @encoderEventListener;
            %obj.odoPosition = [1;0;0];
            pause(0.1);
            
            
            %CONTROLER Construct an instance of this class
            %   Detailed explanation goes here
            
            disp(encoderDataTimeStamp);
        end
        function [obj, odoX, odoY, odoTh] = backup(obj, dist)
            global encoderFrame;
            global encoderDataReady;
            global encoderDataTimeStamp;
            global encoderDataStarted;
            global encoderDataTimeStart;
            usePid = true;

            bEncoderX = obj.robIF.encoders.LatestMessage.Vector.X;
            bEncoderY = obj.robIF.encoders.LatestMessage.Vector.Y;
            lastEncoderTime = encoderDataTimeStamp;


            v = 0.2;
            encoderX = obj.robIF.encoders.LatestMessage.Vector.X;
            encoderY = obj.robIF.encoders.LatestMessage.Vector.Y;
            x = (encoderX + encoderY) / 2 - (bEncoderY + bEncoderX)/2;
            while abs(x) < dist
                encoderX = obj.robIF.encoders.LatestMessage.Vector.X;
                encoderY = obj.robIF.encoders.LatestMessage.Vector.Y;
                x = (encoderX + encoderY) / 2 - (bEncoderY + bEncoderX)/2;
                obj.robIF.sendVelocity(-v,-v);
                pause(0.01);
            end
            to = obj.robIF.toc()
            while obj.robIF.toc() < 0.5 + to
                obj.robIF.sendVelocity(0,0);
            end
            encoderX = obj.robIF.encoders.LatestMessage.Vector.X;
            encoderY = obj.robIF.encoders.LatestMessage.Vector.Y;
            x = (encoderX + encoderY) / 2 - (bEncoderY + bEncoderX)/2;
            odoX = x;
            odoY = 0;
            odoTh = 0;
            obj.odoPosition = [odoX; odoY; odoTh];

        end
        function [obj, odoX, odoY, odoTh] = run(obj,traj)
            %using pre-generated trajectory

            global encoderFrame;
            global encoderDataReady;
            global encoderDataTimeStamp;
            global encoderDataStarted;
            global encoderDataTimeStart;
            usePid = true;

            lastEncoderX = obj.robIF.encoders.LatestMessage.Vector.X;
            lastEncoderY = obj.robIF.encoders.LatestMessage.Vector.Y;
            lastEncoderTime = encoderDataTimeStamp;
            odoX = 0;
            odoY = 0;
            odoTh = 0;
            tf = max(traj.timeArray) + 0.5;
            beginTime = obj.robIF.toc();

            t = obj.robIF.toc() - beginTime;
            lastT = t;

            refx = 0;
            refy = 0;
            refth = 0;
            obj.records = [0,0, 0, 0, 0, 0, 0, 0, 0, 0];
            obj.vArray = traj.VArray;
            while t < tf - 1
                t = obj.robIF.toc() - beginTime;
                dt = t - lastT;
                lastT = t;
                %[V, w] = refControl.computeControl(t);
                V = interp1(traj.timeArray, traj.VArray, t);
                w = interp1(traj.timeArray, traj.wArray, t);
                %disp(traj(index,:));
                %[refx, refy, refth] = interp1(traj.timeArray, traj.poseArray, t);
                refx = interp1(traj.timeArray, traj.poseArray(1,:), t);
                refy = interp1(traj.timeArray, traj.poseArray(2,:), t);
                refth = interp1(traj.timeArray, traj.poseArray(3,:), t);
                %[refx, refy, refth] = obj.integrate_ref(refx, refy, refth, dt, V, w);

                %disp([xe,ye,the]);
                %disp('\n');
                %pause(0.02);

                %%%% Odometry
                newEncoderX = obj.robIF.encoders.LatestMessage.Vector.X;
                newEncoderY = obj.robIF.encoders.LatestMessage.Vector.Y;
                newEncoderTime = encoderDataTimeStamp;
                encodt = newEncoderTime - lastEncoderTime;
                encodx = newEncoderX - lastEncoderX;%left encoder
                encody = newEncoderY - lastEncoderY;%right encoder
                if encodt == 0
                    pause(0.01);
                else
                    lvel = encodx/encodt;
                    rvel = encody/encodt;
                    [dOdoX, dOdoY, dOdoTh, odoT] = modelDiffSteerRobot(lvel, rvel, odoTh, lastEncoderTime, newEncoderTime, encodt);
                    odoX = odoX + dOdoX;
                    odoY = odoY + dOdoY;
                    odoTh = odoTh + dOdoTh;
                    lastEncoderX = newEncoderX;
                    lastEncoderY = newEncoderY;
                    lastEncoderTime = newEncoderTime;
                end
                
                
                conversionMatrix = inv([cos(odoTh), -sin(odoTh); sin(odoTh), cos(odoTh)]);
                exw = refx - odoX;
                eyw = refy - odoY;
                ethw = refth - odoTh;
                exyr = conversionMatrix * [exw; eyw];
                
                gain = 4;
                exr = exyr(1);
                eyr = exyr(2);
                if usePid
                    uv = exyr(1) * gain;
                    uw = exyr(2) * gain + ethw * gain;
                    [vl, vr] = robotUtil.VwTovlvr(V + uv, w + uw);
                    
                else
                    
                    [vl, vr] = robotUtil.VwTovlvr(V, w);
                    vl, vr
                end
                
                
                if vl > 0.3
                    vl = 0.3;
                end
                if vl < -0.3
                    vl = -0.3;
                end
                if vr > 0.3
                    vr = 0.3;
                end
                if vr < -0.3
                    vr = -0.3;
                end
                obj.robIF.sendVelocity(vl, vr);
                obj.records(length(obj.records(:,1)) + 1, :) = [refx, refy,refth, odoX, odoY, odoTh, t, exr, eyr, ethw];
                obj.odoPosition = [odoX; odoY; odoTh];
                pause(0.01);
            end
            disp(obj.odoPosition);
            tic;
            while toc < 0.5
                obj.robIF.sendVelocity(0,0);
                pause(0.1);
                %disp("segment done, stoping");
            end
            
%             hold on
%             plot(records(:, 1), records(:,2), '-r');
%             plot(records(:, 4), records(:,5),'-k');
%             hold off
%             
        end
        
        function [x,y,th] = integrate_ref(obj, x, y, th, dt, V, w)
            th = th + w/2 * dt;
            dx = V * cos(th) * dt;
            dy = V * sin(th) * dt;
            th = th + w/2 * dt;
            x = x + dx;
            y = y + dy;
        end
    end
end

