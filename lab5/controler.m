

classdef controler
    %CONTROLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robIF
        
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
            pause(0.1);
            
            
            %CONTROLER Construct an instance of this class
            %   Detailed explanation goes here
            
            disp(encoderDataTimeStamp);
        end
        
        function outputArg = run(obj,refControl, traj)
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
            tf = refControl.getTrajectoryDuration();
            beginTime = obj.robIF.toc();

            t = obj.robIF.toc() - beginTime;
            lastT = t;

            refx = 0;
            refy = 0;
            refth = 0;
            records = [0,0, 0, 0, 0, 0, 0, 0, 0, 0];
            while t < tf
                t = obj.robIF.toc() - beginTime;
                dt = t - lastT;
                lastT = t;
                [V, w] = refControl.computeControl(t);
                

                index = 1;
                while traj(index, 1) < t && index < length(traj(:, 1))
                    index = index + 1;
                end
                %disp(traj(index,:));
                xe = traj(index, 2);
                ye = traj(index, 3);
                the = traj(index,4);
                [refx, refy, refth] = obj.integrate_ref(refx, refy, refth, dt, V, w);

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
                records(length(records(:,1)) + 1, :) = [refx, refy,refth, odoX, odoY, odoTh, t, exr, eyr, ethw];
                pause(0.01);
            end
            tic;
            while toc < 1
                obj.robIF.sendVelocity(0,0);
                pause(0.1);
            end
            hold on
            plot( records(:, 7), records(:, 1));
            plot( records(:, 7), records(:, 2));
            plot( records(:, 7), records(:, 3));
            plot( records(:, 7), records(:, 4));
            plot( records(:, 7), records(:, 5));
            plot( records(:, 7), records(:, 6));

            hold off
            figure();
            hold on
            plot(records(:, 1), records(:,2));
            plot(records(:, 4), records(:,5));
            hold off
            figure();
            hold on;
            plot( records(:, 7), records(:, 8));
            plot( records(:, 7), records(:, 9));
            plot( records(:, 7), records(:, 10));
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

