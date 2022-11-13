classdef l8pointCloud < rangeImage
    %pointCloud Creates a point cloud from a range image.
    %
    %   A "pointCloud" is just a set of points that comes from converting a
    %   range image to cartesian (initially sensor- or image- frame
    %   referenced) coordinates. Related services include subsampling the
    %   data to reduce density, to throw out bad points, or to throw out
    %   data that is outside a region of interest. 
    %
    %   For ease of both plotting and processing as 2D points, all derived
    %   data is laid out as row vectors. This makes it easy to lay out a
    %   matrix whose rows are all x coordinates etc. to convert coordinates
    %   of all lidar points in one matrix multiply.
    %
    %   In 3D vision, processing a few million range points per second
    %   might make you want to multiply all the transform matrices together
    %   before converting points to the vehicle or the world frame but that
    %   does not matter here.
    %
    %   CAUTION: The range image as provided directly by ROS is a column vector.
    %
    %   In order to avoid propagation of dependence on a sensor model
    %   throughout the code, this class converts incoming data to the robot
    %   frame automatically in the constructor. This class inherits from
    %   rangeImage and rangeSensor so that the geometry of the pixel layout
    %   and pose of the sensor on-the-robot becomes known here.
    %
    %   In this system, there is little to no value in processing the range
    %   image directly, so it is good practice to convert range images to
    %   point clouds immediately when they arrive and then proceed with the
    %   point cloud. While the points in the cloud are present and will
    %   remain in the order of the original range pixels, the numerous
    %   filtering steps as well as varying surface incidence angles make it
    %   a bad idea to rely on any particular point point density in angle
    %   or in space.
    %
    %   $Author: AlonzoKelly $  $Date: 2020/07/15 14:00:00 $ $Revision: 2.0 $
    %   Copyright: Alonzo Kelly 2020
    %
    
    properties(Constant)
    end
    
    properties(Access = public)
        xSenArray = []; % array of x'es in sensor coordinates
        ySenArray = []; % array of y'es in sensor coordinates
        
        skip;       % number of pixels to skip before processing next one
        cleanFlag;  % remove near and far points
    end

    properties(Access = public)
        xArray = []; % array of x'es in robot coordinates
        yArray = []; % array of y'es in robot coordinates
        robotLidarPts = []; % the homogeneous points [xArray ; yArray ; ones(1,len)]
    end
    
    methods(Static = true)        
        function testClass
            ranges = zeros(360,1);
            pointCloud(ranges);
            pointCloud(ranges,[1;2;3]);
            pointCloud(ranges,3,true);
            pointCloud(ranges,[1;2;3],3,true);
        end
    end
    
    methods(Access = private)
        
    end
            
    methods(Access = public)
        
        function obj = l8pointCloud(ranges,varargin)
            %pointCloud Constructs an instacnce of this class.
            % 
            %   pointCloud() will construct an empty pointCloud. This version
            %   is useful if you want the set the sensorPose before calling
            %   setRanges.
            %
            %   pointCloud(ranges) will construct a pointCloud for the row
            %   vector ranges.
            %
            %   pointCloud(ranges,skip,cleanFlag) will construct an image of
            %   subsampled data and then also remove points outside the range
            %   window [maxUsefulRange minUsefulRange] if cleanFlag is set.
            %
            %   CAUTION: The range image as provided directly by ROS is a
            %   column vector. This class assumes that the provided ranges
            %   is a column vector and it transposes that data imeidately
            %   as a first step.
            %
            obj@rangeImage(varargin{:});
                      
            obj.setFilters(1,false); % defaults
            
            if(nargin == 0); return; end

            if(~all(size(ranges) == [360 1]))
                error("Raw range image must be 360 X 1");
            end
                                        
            % Stuff the arrays         
            if(nargin >= 2)        
                obj.skip = varargin{1};
            end    
            if(nargin >= 3)
                obj.cleanFlag = varargin{2};
            end
            setRanges(obj,ranges);
        end  
                
        function setFilters(obj,skip,cleanFlag)
            %setFilters Sets the data sampling and filtering parameters.
            %
            %   obj.setFilters(skip,cleanFlag) skip is the number of pixels
            %   to skip to get to the next one used and cleanFlag will
            %   cause near and far points to be eliminated from the point cloud.
            %
            obj.skip = skip;
            obj.cleanFlag = cleanFlag;
        end
        
        function setRanges(obj,ranges)
            %setRanges sets the range data and recomputes the point cloud.
            %
            %   obj.setRanges(ranges) will set the range values and
            %   recompute the point cloud as indicated by the prevailing
            %   filter settings.
            %
            %   CAUTION: The range image as provided directly by ROS is a
            %   column vector. This class assumes that the provided ranges
            %   is a column vector and it transposes that data imeidately
            %   as a first step.
            %
            ranges = ranges'; % convert from column to row;
            % Indices
            num = pointCloud.numRawPixels/obj.skip;
            obj.iArray = round(linspace(1,pointCloud.numRawPixels,num));
            % Generate, adjust, and normalize angles
            obj.tArray = (obj.iArray-1)*(pi/180)-5*(pi/180);
            obj.tArray = atan2(sin(obj.tArray),cos(obj.tArray)); % force into canonical angles
            obj.rArray = ranges(obj.iArray); % picks off the nonskipped ranges
            % X and y 
            obj.xSenArray = obj.rArray .* cos(obj.tArray);
            obj.ySenArray = obj.rArray .* sin(obj.tArray);

            if obj.cleanFlag; obj.removeBadPoints(); end
            
            % Convert only x and y to robot coordinates in one shot with
            % homogeneous matrix multiplication
            sensorLidarPts = [obj.xSenArray ; obj.ySenArray ; ones(1,obj.numPixels)];
            obj.robotLidarPts = obj.getSenToRob()*sensorLidarPts;
            obj.xArray = obj.robotLidarPts(1,:);
            obj.yArray = obj.robotLidarPts(2,:);
        end
        
        function removeBadPoints(obj)
            %removeBadPoints removes near and far data.
            %
            % obj.removeBadPoints() Takes all points above and below two
            % range thresholds out of the point arrays. This is a convenience but
            % it should not be used by any routine that expects the resulting points
            % to be equally separated in angle. The operation is done
            % inline and removed data is deleted from the original arrays.
            %
            goodOnes = obj.rArray > pointCloud.minUsefulRange & obj.rArray < pointCloud.maxUsefulRange;

            obj.iArray = obj.iArray(goodOnes);
            obj.tArray = obj.tArray(goodOnes);
            obj.rArray = obj.rArray(goodOnes);
            obj.xSenArray = obj.xSenArray(goodOnes);
            obj.ySenArray = obj.ySenArray(goodOnes);
 
            if(isempty(goodOnes))
                fprintf('pointCloud: All data in image is BAD !!\n');
            end
        end
                        
        function plotXvsY(obj, maxRange)
            % plotXvsY Plots the point cloud in cartesian coordinates
            %
            %   obj.plotXvsY(maxRange) Draws the cloud in the currently
            %   active figure. Plots the range image in sensor coordinates.
            %
            plotThetas = obj.tArray;
            plotRanges = obj.rArray;
            plot(plotRanges.*cos(plotThetas),plotRanges.*sin(plotThetas),'r.');
            axis equal
            axis([-maxRange maxRange -maxRange maxRange])
            grid on
        end
        
        function bool = isEmpty(obj)
            % isEmpty Test if there is any data in the cloud.
            %
            %   obj.isEmpty() Returns true if there is no data left. This
            %   can happen if your filtering is too agressive or if all of
            %   the data points are bad.
            %
            bool = numPixels(obj) == 0;
        end
       
        function num = numPixels(obj)
            % numPixels Returns the number of pixels in the cloud.
            %
            %   obj.numPixels() Returns the number of pixels (points) in
            %   the point cloud.
            %
            num = length(obj.iArray);
        end

%         function out = inc(obj,in)
%             % increment with wraparound over natural numbers
%             out = indexAdd(obj,in,1);
%         end
%         
%         function out = dec(obj,in)
%             % decrement with wraparound over natural numbers
%             out = indexAdd(obj,in,-1);
%         end
%         
%         function out = indexAdd(obj,a,b)
%             % add with wraparound over natural numbers. First number
%             % “a” is "natural" meaning it >=1. Second number is signed. % Convert a to 0:3 and add b (which is already 0:3).
%             % Convert the result back by adding 1.
%             
%             out = mod((a-1)+b,obj.numPixels())+1;
%         end
    end
end