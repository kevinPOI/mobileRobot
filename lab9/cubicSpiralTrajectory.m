classdef cubicSpiralTrajectory < handle
    %cubicSpiral A planar trajectory generator.
    %
    %   Implements a planar trajectory specified in terms of three 
    %   coefficients that adjust the terminal pose of the robot. The initial
    %   pose is assumed to be the origin with zero curvature. The terminal
    %   curvature is forced to be zero.
    %
    %   $Author: AlonzoKelly $  $Date: 2020/07/15 14:00:00 $ $Revision: 2.0 $
    %   Copyright: Alonzo Kelly 2020
    %
    
    properties(Constant)

    end
    
    properties(Access = private)
        parms = [0 0 1];
        sgn=0.0;
        rampLength = 0.05;
    end
    
    properties(Access = public)
        numSamples = 0;
        distArray = [];
        timeArray = [];
        poseArray = [];
        curvArray = [];
        vlArray = []
        vrArray = []
        VArray = [];
        wArray = [];
    end
    
    methods(Static = true)
        
        function makeLookupTable(scale)
            %MAKELOOKUPTABLE Make 6 lookup tables to be used for cubicSpiral generation.
            %
            % The principle is to sample densely in coefficient space and
            % integrate the curvature to find the curve, then store the
            % result in a table that inverts the mapping.
            %
            % On a 2.6 GHz core i7:
            % Scale = 100 takes 0.17 minutes
            % Scale = 50 takes 0.64 minutes
            % Scale = 10 takes 15 minutes
            % Scale = 2 takes 381 minutes
            % Scale = 1 takes > 125 hours (and still needs interpolation)
            
            startTic = tic();

            sMax = 1.0;   % length of curve
            qMax =  pi(); % max bearing
            qMin = -pi(); % min bearing
            tMax =  1.5*pi(); % max heading
            tMin = -1.5*pi(); % min heading

            numT = round(10*126/scale); % heading samples
            numQ = round(50*126/scale); % bearing samples

            % Do a quick performance estimate
            radius = 1.0;
            ds = radius*(qMax-qMin)/numQ;
            dt = (tMax-tMin)/numT;
            fprintf('Error predicted %f m in translation, %f rads in rotation\n',ds,dt);

            % Allocate tables
            a1Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT);
            b1Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT);
            r1Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT); % energy integral

            a2Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT);
            b2Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT);
            r2Tab = scalarField(qMin,qMax,numQ,tMin,tMax,numT); % energy integral

            clothSamples = 201;

            plotSamples = 10000;
            plotArrayX = zeros(plotSamples);
            plotArrayY = zeros(plotSamples);

            n = 1;
            aMax = 195.0/sMax/sMax;
            bMax = 440.0/sMax/sMax/sMax;

            % 15 minutes at scale = 10
            % X at scale = 1
            numA = 40000.0/scale;
            numB = 25000.0/scale;
            for a = -aMax:aMax/numA:aMax
                for b = -bMax:bMax/numB:bMax
                    ds = sMax/(clothSamples-1); 
                    x=0.0; y = 0.0; t = 0.0; r = 0.0;sf = 1;s = 0;
                    broke = false;
                    for i=1:clothSamples %FILL IN ______________________________________________________
                        broke = false;
                        fs = s * (a + b * s)*(s - sf);
                        r = r + fs^2 * ds;
                        t = t + fs * ds;
                        x = x + cos(t)*ds;
                        y = y + sin(t)*ds;
                        s = s + ds;
                        obj.poseArray(1, i) = x;
                        obj.poseArray(2, i) = y;
                        obj.poseArray(3, i) = t;
                        if abs(t) > abs(tMax)
                            broke = true;
                            break;
                        end
                    % Compute the curve. Break out of this loop, and then 
                    % immediately continue to next iteration of the for b loop 
                    % if tmax is exceeded in absolute value at any time.
					% FILL THIS IN
                    end
                    if(broke == true); continue; end

                    q = atan2(y,x);
                    %if(~aTab.isInBounds(q,t)); continue; end;
                    % This is faster
                    if(q<qMin || q>=qMax || t<tMin || t>=tMax) ; continue; end
                    if(n <= plotSamples)
                        plotArrayX(n) = x;
                        plotArrayY(n) = y;
                    elseif(n == plotSamples+1)
                        if(scale > 20)
                            figure(1);
                            plot(plotArrayX(1:n-1),plotArrayY(1:n-1),'.k','MarkerSize',3);
                            hold on;
                            fprintf('Plot array dumping\n');
                        end
                        n = 0;
                        elapsedTime = toc(startTic);
                        fprintf('Took %f minutes\n',elapsedTime/60.0);
                    end
                    n = n + 1;
                    % Store coefficients.
                    r1Now = r1Tab.get(q,t);
                    r2Now = r2Tab.get(q,t);
                    if(abs(r1Now) > abs(r) && a >=0.0)
                        r1Tab.set(q,t,r);
                        a1Tab.set(q,t,a);
                        b1Tab.set(q,t,b);
                    elseif(abs(r2Now) > abs(r) && a <=0.0)
                        r2Tab.set(q,t,r);
                        a2Tab.set(q,t,a);
                        b2Tab.set(q,t,b);
                    end
                end

            end

            elapsedTime = toc(startTic);
            fprintf('Took %f minutes\n',elapsedTime/60.0);
            
            if(scale > 10)
                plot(plotArrayX(1:n-1),plotArrayY(1:n-1),'.k','MarkerSize',3);
            end
            
            save('cubicSpirals','a1Tab','a2Tab','b1Tab','b2Tab','r1Tab','r2Tab');

            figure(2);
            I1 = mat2gray(a1Tab.cellArray, [-aMax aMax]);
            imshow(I1);
            xlabel('Heading'); ylabel('Bearing'); title('a1Tab');
            figure(3);
            I2 = mat2gray(a2Tab.cellArray, [-aMax aMax]);
            imshow(I2);
            xlabel('Heading'); ylabel('Bearing'); title('a2Tab');
            figure(4);
            I1 = mat2gray(b1Tab.cellArray, [-bMax bMax]);
            imshow(I1);
            xlabel('Heading'); ylabel('Bearing'); title('b1Tab');
            figure(5);
            I2 = mat2gray(b2Tab.cellArray, [-bMax bMax]);
            imshow(I2);
            xlabel('Heading'); ylabel('Bearing'); title('b2Tab');
        end
        
        function curve = planTrajectory(x,y,th,sgn)
        %PLANTRAJECTORY Uses the lookup tables to generate a cubic spiral trajectory
        % ending at the desired pose.
        %
        %   curve = PLANTRAJECTORY(x, y, theta, sgn) returns the cubic
        %   spiral trajectory curve ending at coordinate (x,y) with
        %   orientation theta.  The sign of the trajectory sgn indicates
        %   whether the robot should drive forwards or backwards.
            persistent inited;
            persistent a1T a2T b1T b2T r1T r2T;
                    
            if(isempty(inited))
                load('cubicSpirals.mat','a1Tab','a2Tab','b1Tab','b2Tab','r1Tab','r2Tab');
                inited = true;
                a1T = a1Tab;a2T = a2Tab;b1T = b1Tab;b2T = b2Tab;r1T = r1Tab;r2T = r2Tab;
            end
            
            q = atan2(y,x);
            % set pi(0 equal to - pi() to avoid out of bounds in scalar
            % field.
            if(abs(q-pi()) < 1e-10) ; q = -pi(); end
            t = th;
            a1 = a1T.get(q,t);
            a2 = a2T.get(q,t);
            b1 = b1T.get(q,t);
            b2 = b2T.get(q,t);
            r1 = r1T.get(q,t);
            r2 = r2T.get(q,t);

            oneBad = false;
            twoBad = false;
            if(isinf(a1) || isinf(b1))
                oneBad = true;
            end
            if(isinf(a2) || isinf(b2))
                twoBad = true;
            end

            % Pick sole good one or the least curvy
            % I tried with and without this and it makes a huge difference! 
            % Its the secret to computing a table in reasonable time because the
            % least curvy is pretty accurate
            if(oneBad==true && twoBad==true)
                disp('No solution\n');
                curve = {};
                return;
            elseif(oneBad==true)
                au=a2;bu=b2;
            elseif(twoBad==true)
                au=a1;bu=b1;       
            elseif(r2 > r1)
                au=a1;bu=b1;  
            else
                au=a2;bu=b2;
            end

            % Plot the corresponding unit
            su = 1.0;
            clothu = cubicSpiralTrajectory([au bu su],201);

            %hold on;

            % Scale it up to the original
            xu = clothu.poseArray(1,clothu.numSamples);
            yu = clothu.poseArray(2,clothu.numSamples);
            Ru = sqrt(xu*xu+yu*yu);
            RO = sqrt(x*x+y*y);
            lam = RO/Ru;

            ss = lam;
            as = au/lam/lam/lam;
            bs = bu/lam/lam/lam/lam;
            
            if(sgn < 0)
                as = -as;  
                ss = -ss;
            end
            curve = cubicSpiralTrajectory([as bs ss],201);
        end
            
    end
    
    methods(Access = private)
        
       function setParms(obj,parms)
       %SETPARMS Sets the parameters of the cubic spiral.
       %
       %    obj.SETPARMS(parms) sets the parameters to those indicated by
       %    the 3 x 1 vector parms.
            obj.parms = parms;
        end
        
        function [x,y,th] = integrate_ref(obj, x, y, th, dt, V, w)
            th = th + w/2 * dt;
            dx = V * cos(th) * dt;
            dy = V * sin(th) * dt;
            th = th + w/2 * dt;
            x = x + dx;
            y = y + dy;
        end

        function integrateCommands(obj)
        %INTEGRATECOMMANDS Integrates the velocity commands for the cubic
        % spiral trajectory.
            len = obj.numSamples;
            obj.distArray  = zeros(1,len);
            obj.poseArray  = zeros(3,len);
            obj.curvArray  = zeros(1,len);

            % Place robot in initial state
            obj.distArray(1) = 0.0;
            obj.poseArray(1,1) = 0.0;
            obj.poseArray(2,1) = 0.0;
            obj.poseArray(3,1) = 0.0;
            obj.curvArray(1) = 0.0;

            a = obj.parms(1);
            b = obj.parms(2);
            sf = obj.parms(3);
            ds = sf/(obj.numSamples-1);
            s = 0;
            th = 0;
            x = 0;
            y = 0;
            dist = 0;
            for i=2:obj.numSamples
                %fil in here%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 vl = obj.vlArray(i);
%                 vr = obj.vlArray(i);
%                 [V, w] = robotUtil.vlvrToVw(vl, vr);
%                 x = obj.poseArray(1,i - 1);
%                 y = obj.poseArray(2, i - 1);
%                 th = obj.poseArray(3, i - 1);
%                 dt = obj.timeArray(i) - obj.timeArray(i - 1);
%                 [x, y, th] = obj.integrate_ref(x, y, th, dt, V, w);
%                 obj.poseArray(1, i - 1) = x;
%                 obj.poseArray(2, i - 1) = y;
%                 obj.poseArray(3, i - 1) = th;
                fs = s * (a + b * s)*(s - sf);
                th = th + fs * ds;
                x = x + cos(th)*ds;
                y = y + sin(th)*ds;
                dist = dist + sqrt((cos(th) * ds)^2 + (sin(th)*ds)^2);
                s = s + ds;
                obj.poseArray(1, i) = x;
                obj.poseArray(2, i) = y;
                obj.poseArray(3, i) = th;
                obj.curvArray(i) = fs;
                obj.distArray(i) = dist;

            end
            i = obj.numSamples;
            s = (i-1)*ds;  
            obj.distArray(i) = s;
            obj.curvArray(i) = 0.0;
        end   
        
        function computeTimeSeries(obj)
        %COMPUTETIMESERIES Finds the times implied by the distances and the
        % velocities.
            len = obj.numSamples;
            obj.timeArray  = zeros(1,len);

            % Place robot in initial state
            obj.timeArray(1) = 0.0;

            for i=1:obj.numSamples-1
                ds = obj.distArray(i+1) - obj.distArray(i);
                V = obj.VArray(i);
                % Avoid division by zero
                if(abs(V) < 0.001); V = obj.VArray(i+1); end

                obj.timeArray(i+1)= obj.timeArray(i)+ds/abs(V);
            end
        end   
    end
            
    methods(Access = public)
        
        function obj = cubicSpiralTrajectory(parms,numSamples)
            %CUBICSPIRAL Constructs a cubicSpiral for the supplied
            % parameters.  The curve is integrated immediately when this
            % constructor is called. Thereafter the public arrays 
            % distArray, poseArray, and curvArray are populated.
            %
            %   spiral = CUBICSPIRAL(parms, n) creates a cubic spiral
            %   spiral for the parameters parms = [a b sf].  n represents
            %   the number of integration steps used to integrate the
            %   entire curve.
            if(nargin == 2)
                obj.numSamples = numSamples;
                obj.parms = parms;
                obj.integrateCommands();
                obj.sgn = sign(parms(3)); % Critical for proper time series
            end
        end
        
        function plot(obj)
        %PLOT Plots the pose array for the cubic spiral.
            plotArray1 = obj.poseArray(1,:);
            plotArray2 = obj.poseArray(2,:);
            plot(plotArray1,plotArray2,'r');
            xf = obj.poseArray(1,obj.numSamples);
            yf = obj.poseArray(2,obj.numSamples);
            r = max([abs(xf) abs(yf)]);
            xlim([-2*r 2*r]);
            ylim([-2*r 2*r]);
        end      
                
       function planVelocities(obj,Vmax)
       %PLANVELOCITIES Plans velocities for the path.
       %
       %    obj.PLANVELOCITIES(Vmax) plans the highest velocities possible
       %    for the cubic spiral path while ensuring that the absolute
       %    value of all wheel velocities remains below Vmax.
            for i=1:obj.numSamples
                Vbase = Vmax;
                % Add velocity ramps for first and last 5 cm
                s = obj.distArray(i);
                sf = obj.distArray(obj.numSamples);
                if(abs(sf) > 2.0*obj.rampLength) % no ramp for short trajectories
                    sUp = abs(s);
                    sDn = abs(sf-s);
                    if(sUp < obj.rampLength) % ramp up
                        Vbase = Vbase * sUp/obj.rampLength;
                    elseif(sDn < 0.05) % ramp down
                        Vbase = Vbase * sDn/obj.rampLength;
                    end
                end
                % Now proceed with base velocity 
                %disp(Vbase);
                V = Vbase*obj.sgn; % Drive forward or backward as desired.
                K = obj.curvArray(i);
                w = K*V;
                vr = V + robotModel.W2*w;
                vl = V - robotModel.W2*w;               
                if(abs(vr) > Vbase)
                    vrNew = Vbase * sign(vr);
                    vl = vl * vrNew/vr;
                    vr = vrNew;
                end
                if(abs(vl) > Vbase)
                    vlNew = Vbase * sign(vl);
                    vr = vr * vlNew/vl;
                    vl = vlNew;
                end
                obj.vlArray(i) = vl;
                obj.vrArray(i) = vr;
                obj.VArray(i) = (vr + vl)/2.0;
                obj.wArray(i) = (vr - vl)/robotModel.W;                
            end
            % Now compute the times that are implied by the velocities and
            % the distances.
            obj.computeTimeSeries();
        end
        
        function J = getJacobian(obj)
        %GETJACOBIAN Returns the Jacobian matrix for the cubic spiral.
            eps = 0.00001;
            dp = [eps 0.0 0.0];
            clothPlus1 = cubicSpiral(obj.parms+dp,obj.numSamples);
            posePlus = clothPlus1.getFinalPose;
            poseObj = obj.getFinalPose();
            J(:,1) = (posePlus-poseObj)/eps;
           
            dp = [0.0 eps 0.0];
            clothPlus2 = cubicSpiral(obj.parms+dp,obj.numSamples);
            posePlus = clothPlus2.getFinalPose;
            poseObj = obj.getFinalPose();
            J(:,2) = (posePlus-poseObj)/eps;
            
            dp = [0.0 0.0 eps];
            clothPlus3 = cubicSpiral(obj.parms+dp,obj.numSamples);            
            posePlus = clothPlus3.getFinalPose;
            poseObj = obj.getFinalPose();
            J(:,3) = (posePlus-poseObj)/eps; 
        end
        
        function refineParameters(obj,goalPose)
        %REFINEPARAMETERS Recomputes the parameters of a cubic spiral path
        % to drive the final pose of the cubic spiral to the goal pose for
        % the path.
            disp('Entering iterations');
            newParms = obj.getParms();
            newObj = cubicSpiral(newParms,5001); % hi samples for derivatives
            thisPose = newObj.getFinalPose();
            error = inf; n = 1;
            while(error > 0.001)
                %disp('desired pose');
                %disp(goalPose);
                J = newObj.getJacobian();
                if(rcond(J) < 1.0e-6)
                    fprintf('Unable to invert Jacobian. Exiting\n');
                    break;
                end
                delP = J^-1 * (goalPose-thisPose);
                newParms = newParms+delP';
                newObj.setParms(newParms);
                newObj.integrateCommands();
                %disp('this pose');
                thisPose = newObj.getFinalPose();
                error = norm(thisPose-goalPose);
                disp(error);
                if(n >=20)
                    fprintf('cubicSpiral failing to refine Parameters\n');
                    break;
                end
            end
        end
        
        function s  = getDistAtDist(obj,s)
        %GETDISTATDIST Returns some distance, I don't know why
            if( s < obj.distArray(1))
                s = 0.0;
            else
                s  = interp1(obj.distArray,obj.distArray,s,'pchip','extrap');  
            end
        end
        
        function K  = getCurvAtDist(obj,s)
        %GETCURVATDIST Returns the curve of the cubic spiral at a distance
        % along the path.
        %
        %   k = obj.GETCURVATDIST(s) returns the curve k of the cubic
        %   spiral at a distance s along the path.
            if( s < obj.distArray(1))
                K = 0.0;
            else
                K  = interp1(obj.distArray,obj.curvArray,s,'pchip','extrap');  
            end
        end
            
        function pose  = getPoseAtDist(obj,s)
        %GETPOSEATDIST Returns the pose of the robot at a given distance
        % along a cubic spiral path.
        %
        %   pose = obj.GETPOSEATDIST(s) returns the pose of the robot in
        %   the form pose = [x; y; theta] at a distance s along the cubic
        %   spiral path.
            x = interp1(obj.distArray,obj.poseArray(1,:),s,'pchip','extrap');
            y = interp1(obj.distArray,obj.poseArray(2,:),s,'pchip','extrap');
            th = interp1(obj.distArray,obj.poseArray(3,:),s,'pchip','extrap');
            pose  = [x ; y ; th];  
        end
        
        function pose  = getFinalPose(obj)
        %GETFINALPOSE Returns the final pose of the robot after it has
        % followed the cubic spiral path.
            pose  = obj.poseArray(:,obj.numSamples);  
        end  
        
        function time  = getTrajectoryDuration(obj)
        %GETTRAJECTORYDURATION Returns the amount of time required for the
        % robot to traverse the cubic spiral path.
            time  = obj.timeArray(:,obj.numSamples);  
        end
        
        function dist  = getTrajectoryDistance(obj)
        %GETTRAJECTORYDISTANCE Returns the total distance covered by the
        % cubic spiral path.
            dist  = obj.distArray(:,obj.numSamples);  
        end
        
        function V  = getVAtTime(obj,t)
        %GETVATTIME Returns the planned linear velocity for the robot at a
        % given time.
        %
        %   v = obj.GETVATTIME(t) returns the intended linear velocity v
        %   for the robot at time t as it travels along the cubic spiral
        %   path.
            if( t < obj.timeArray(1))
                V = 0.0;
            else
                V  = interp1(obj.timeArray,obj.VArray,t,'pchip','extrap');  
            end
        end
            
        function w  = getwAtTime(obj,t)
        %GETWATTIME Returns the planned angular velocity for the robot at a
        % given time.
        %
        %   w = obj.GETWATTIME(t) returns the intended angular velocity w
        %   for the robot at time t as it travels along the cubic spiral
        %   path.
            if(t < obj.timeArray(1))
                w = 0.0;
            else
                w  = interp1(obj.timeArray,obj.wArray,t,'pchip','extrap');  
            end
        end
            
        function pose  = getPoseAtTime(obj,t)
        %GETPOSEATTIME Returns the planned pose of the robot at a given
        % time.
        %
        %   pose = obj.GETPOSEATTIME(t) returns the intended pose of the
        %   robot in the form pose = [x; y; theta] at time t as the robot
        %   traverses a cubic spiral path.
            x = interp1(obj.timeArray,obj.poseArray(1,:),t,'pchip','extrap');
            y = interp1(obj.timeArray,obj.poseArray(2,:),t,'pchip','extrap');
            th = interp1(obj.timeArray,obj.poseArray(3,:),t,'pchip','extrap');
            pose  = [x ; y ; th];  
        end  
        
        function parms  = getParms(obj)
        %GETPARMS Returns the parameters of the cubic spiral path.
            parms  = obj.parms;  
        end
        
        function setSgn(obj,sgn)
        %SETSGN Sets the sign of the cubic spiral path.
        %
        %   obj.SETSGN(1) will cause the robot to travel forwards.
        %
        %   obj.SETSGN(-1) will cause the robot to drive in reverse.
            obj.sgn  = sgn;  
        end  
        
        function reflectX(obj)
        %REFLECTX Reflects the curve around X axis by modifying parameters.
            obj.parms(1) = - obj.parms(1);  
            obj.parms(2) = - obj.parms(2);
            obj.integrateCommands();
        end 
        function reflectY(obj)
        %REFLECTY Reflects the curve around Y axis by modifying parameters.
            obj.parms(2) = - obj.parms(2);  
            obj.parms(3) = - obj.parms(3); 
            obj.integrateCommands();
        end 
        
        function reflectXY(obj)
        %REFLECTXY Reflects the curve around both X and Y axes (i.e.
        % through the origin) by modifying parameters.
            obj.parms(1) = - obj.parms(1);  
            obj.parms(3) = - obj.parms(3);
            obj.integrateCommands();
        end 
    end
end