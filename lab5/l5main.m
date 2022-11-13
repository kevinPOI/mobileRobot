global encoderFrame;
global encoderDataReady;
global encoderDataTimeStamp;
global encoderDataStarted;
global encoderDataTimeStart;

%encoder and odometrysetup
encoderDataStarted = false;
robIF = robotIF(0, true);


fig8 = figure8ReferenceControl(3,1,0.5);
traj = robotTrajectory(fig8);

ct = controler(robIF);
ct.run(fig8, traj.samplePoints);