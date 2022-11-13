robIF = robotIF(0, true);
% Create a pallet to look at when in simulation
poseNo = 3;
if(true)
    switch poseNo
        case 1
            palletPose = [0.45 ; 0.00 ; 0.0];
        case 2
            palletPose = [0.45 ; 0.05 ; atan2(0.05,0.45)];%facing
        case 3
            palletPose = [0.45 ; 0.15 ; atan2(0.15,0.45)];
    end
    palletPose(1) = palletPose(1)*(1+0.01*randn);
    palletPose(2) = palletPose(2)*(1+0.01*randn);
    palletPose(3) = palletPose(3)*(1+0.05*randn);
    
    palletShape = palletSailShape(true,palletPose);
    robIF.addObjects(palletShape);

    palletPose = [-0.4 ; -0 ; atan2(0.0,0.45)];
    palletPose(1) = palletPose(1)*(1+0.01*randn);
    palletPose(2) = palletPose(2)*(1+0.01*randn);
    palletPose(3) = palletPose(3)*(1+0.05*randn);
    
    palletShape = palletSailShape(true,palletPose);
    robIF.addObjects(palletShape);
    
    %wall____________________________________________________________
    worldLineArray = worldModel.createThreeWalls();
    worldLineArray2 = [0.6,0.6,-0.6;
                        0.6,-0.3,-0.3;
                        1,1,1];
    
    wallsShape = polyLineShape(worldLineArray2); % create wall
    robIF.addObjects(wallsShape);

end
startLaser(robIF);
pause(1.0);


for i = 1:3
    lidarData = robIF.laser.LatestMessage.Ranges;
    pCloud = l8pointCloud(lidarData);
    pCloud.removeBadPoints;
    [centerX, centerY, orientation] = pointCloudPerceptor.findSail(pCloud);
    [centerX, centerY, orientation]
    
    centerX = centerX - cos(orientation) * 0.080;
    centerY = centerY - sin(orientation) * 0.080;
    mrpl = mrplSystem(robIF);
    
    
    mrpl.goToPoint(0.2, mrpl.x,mrpl.y,mrpl.th,centerX,centerY,orientation,1);
    robIF.forksUp();
    pause(2.0);
    mrpl.getOdoPosition();
    robIF.forksDown();
    
    mrpl.ct.backup(0.15);
    mrpl.getOdoPosition();
    pause(5);
end
% mrpl.goToPoint(0.2, mrpl.x,mrpl.y,mrpl.th,centerX+0.6,centerY+0.2,orientation,1);
% 
% pause(2.0);
% mrpl.getOdoPosition();
% 
% mrpl.goToPoint(0.2, mrpl.x,mrpl.y,mrpl.th,0,0,0,1);
% mrpl.getOdoPosition();
% 
% lidarData = robIF.laser.LatestMessage.Ranges;
% pCloud = l8pointCloud(lidarData);
% pCloud.removeBadPoints;
% [centerX, centerY, orientation] = pointCloudPerceptor.findSail(pCloud);
% [centerX, centerY, orientation]
% centerX = centerX - cos(orientation) * 0.075;
% centerY = centerY - sin(orientation) * 0.075;
% mrpl.goToPoint(0.2, mrpl.x,mrpl.y,mrpl.th,centerX,centerY,orientation,1);
% robIF.forksUp();
% pause(2.0);




