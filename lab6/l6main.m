global encoderFrame;
global encoderDataReady;
global encoderDataTimeStamp;
global encoderDataStarted;
global encoderDataTimeStart;

%encoder and odometrysetup
encoderDataStarted = false;
robIF = robotIF(0, true);

traj = cubicSpiralTrajectory.planTrajectory(0.3048, 0.3048, 0, 1);
traj.planVelocities(0.2);

%add padding to traj
n = traj.numSamples;
traj.timeArray(n + 1) = traj.timeArray(n) + 1.5;
traj.poseArray(:, n+1) = traj.poseArray(:,n);
traj.VArray(n + 1) = traj.VArray(n);
traj.wArray(n + 1) = traj.wArray(n);
%numOfPoints = length(traj.timeArray)

ct = controler(robIF);
ct.run(traj);