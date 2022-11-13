classdef botUtil
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        Property1
    end

    methods(Static)
        function obj = botUtil()
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            
        end

        function outputArg = method1(inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end

        function state = irToXy(i, r)
            theta = i - 9;
            if theta < 0
                theta = theta + 360;
            end

            x = r * cosd(theta);
            y = r * sind(theta);
            state = [x,y,theta];
        end

        function cord = nearestPosition(radarData)
            
            for i = 1 : length(radarData)
                dist = radarData(i);
                if dist < 0.06
                    radarData(i) = 1;
                end
            end
            [r, index] = min(radarData);
            if r > 3
                disp("no valid data")
                cord = [0,0,0];
            else

                state = botUtil.irToXy(index, r);
                cord = state(1:3);
            end
        end

        function velocity = calcVelocity(cord)
            idealRange = 0.5;
            maxSpeed = 0.2;
            dist = sqrt(cord(1)^2 + cord(2)^2);
            if idealRange < dist
                thr = max((dist - idealRange), -1);
            else
                
                thr = min((dist - idealRange), 1);
            end
            theta = cord(3);
            if theta == 0
                turn = 0;
            elseif theta > 0 && theta < 180
                turn = theta * 0.01;
            elseif theta >= 180 && theta < 361
                turn = -(360 - theta) * 0.01;
            else
                turn = 0;
                disp("wrong theta")
                disp(theta)
            end
            lcorrection = -maxSpeed * turn * thr / 2;
            rcorrection = maxSpeed * turn * (1 + thr) / 2;
            lVelocity = maxSpeed * thr + lcorrection;
            rVelocity = maxSpeed * thr + rcorrection;
            if lVelocity > 0.3
                lVelocity = 0.29
            end
            if lVelocity < -0.3
                lVelocity = -0.29
            end
            if rVelocity > 0.3
                rVelocity = 0.29
            end
            if rVelocity < -0.3
                rVelocity = -0.29
            end
            velocity = [lVelocity, rVelocity];
           
        end





                    



    end
end