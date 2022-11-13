robIF = robotIF(0, true);
if robIF.do_sim
 lob = circleShape(0.03,[0.25;0;0]);
 robIF.addObjects(lob);
end
startLaser(robIF);
pause(2.0);
robIF.scr_fig.setEditingMouseHandlers(); 
f = figure('Resize','off');


tic;
while toc < 930

    lidarData = robIF.laser.LatestMessage.Ranges;
    targetCord = botUtil.nearestPosition(lidarData);

    plot(targetCord(1), targetCord(2), "d")
    axis([-2,2,-2,2])
    velocity = botUtil.calcVelocity(targetCord);
    sendVelocity(robIF, velocity(1), velocity(2));
    disp(velocity);
    pause(0.02);
end


