classdef pointCloudPerceptor
    %POINTCLOUDPERCEPTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods
        function obj = pointCloudPerceptor(inputArg1,inputArg2)
            %POINTCLOUDPERCEPTOR Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
    end
    methods(Static = true)    
        function [centerX, centerY, orientation] = findSail(pCloud)
            xArray = pCloud.xSenArray;
            yArray = pCloud.ySenArray;
            xyLambda = [];
            for i=1:length(xArray)
                x = xArray(i);
                y = yArray(i);
                sailX = [];
                sailY = [];
                sailX(1) = x;
                sailY(1) = y;
                
                for j = 1:length(xArray)
                    if j ~= i
                        d2 = (xArray(j) - x)^2 + (yArray(j) - y)^2;
                        if d2 < 0.002
                            sailX(length(sailX) + 1) = xArray(j);
                            sailY(length(sailY) + 1) = yArray(j);
                        end
                    end
                    
                    
                end
                xmean = mean(sailX);
                ymean = mean(sailY);
                Ixx = 0;
                Iyy = 0;
                Ixy = 0;
                for j = 1:length(sailX)
                    Ixx = Ixx + (sailX(j)-xmean)^2;
                    Iyy = Iyy + (sailY(j)-ymean)^2;
                    Ixy = Ixy - (sailX(j)-xmean) * (sailY(j) - ymean);
                end
                Inertia = [Ixx,Ixy; Ixy, Iyy];
                lambda = eig(Inertia);
                lambda = sqrt(lambda)*1000.0;
                th = atan2(2*Ixy, Iyy - Ixx)/2.0;
                if lambda(1) < 1.3
                    xyLambda = [xyLambda; x, y,th, lambda(1), lambda(2)];
                end
                %xyLambda
            
            end
            [minLambda, indice] = min(xyLambda(:,4));%find the "best" point
            candidates = [xyLambda(indice, 1), xyLambda(indice, 2), xyLambda(indice, 3)];
            x = xyLambda(indice, 1);
            y = xyLambda(indice, 2);
            th = xyLambda(indice, 3);
            for i=1:length(xyLambda(:,1))
                d2 = (xyLambda(i,1) - x)^2 + (xyLambda(i,2) - y)^2;
                if d2 < 0.02
                    candidates = [candidates; xyLambda(i,1), xyLambda(i,2), th];
                end
            end
            candidates
            centerX = median(candidates(:, 1));
            centerY = median(candidates(:,2));
            orientation = xyLambda(indice, 3);

        end
    end
end

