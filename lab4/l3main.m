global encoderFrame;
global encoderDataReady;
global encoderDataTimeStamp;
global encoderDataStarted;
global encoderDataTimeStart;


encoderDataStarted = false;
tic;

robIF = robotIF(0, true);
robIF.encoders.NewMessageFcn = @encoderEventListener;
pause(0.1);
lastEncoderX = robIF.encoders.LatestMessage.Vector.X;
lastEncoderY = robIF.encoders.LatestMessage.Vector.Y;
lastEncoderTime = encoderDataTimeStamp;
counter = 1;
velocities = [counter,0];
pause(0.1);

while toc < 0
    
    robIF.sendVelocity(0.1, 0.1);

    newEncoderX = robIF.encoders.LatestMessage.Vector.X;

    newEncoderTime = encoderDataTimeStamp;
    dt = newEncoderTime - lastEncoderTime;
    dx = newEncoderX - lastEncoderX;

    velocity = dx / dt;
    lastEncoderX = newEncoderX;
    lastEncoderTime = newEncoderTime;

    velocities(counter, :) = [counter, velocity];
    counter = counter + 1;
    pause(0.1);
end
beginTime = robIF.toc();
v = 0.2;
sf = 1;
tf = sf / v;
kt = 2 * pi / sf;
kk = 15.1084;
ks = 3;
tf = ks * tf;
T = robIF.toc() - beginTime;
t = T / ks;
odoX = 0;
odoY = 0;
odoTh = 0;
pose = [0,0,0];
while T < tf
    T = robIF.toc() - beginTime;
    t = T / ks;
    s = v * t;
    V = v;
    K = (kk / ks) * sin(kt * s);

    omega = K * V;
    vr = v + robotModel.W / 2 * omega;
    vl = v - robotModel.W / 2 * omega;
    robIF.sendVelocity(vl, vr);

    %%%% Odometry
    newEncoderX = robIF.encoders.LatestMessage.Vector.X;
    newEncoderY = robIF.encoders.LatestMessage.Vector.Y;
    newEncoderTime = encoderDataTimeStamp;
    dt = newEncoderTime - lastEncoderTime;
    dx = newEncoderX - lastEncoderX;
    dy = newEncoderY - lastEncoderY;
    if dt == 0
        pause(0.01);
        %do nothing if no new encoder data
    else
        lvel = dx/dt;
        rvel = dy/dt;
        [dOdoX, dOdoY, dOdoTh, odoT] = modelDiffSteerRobot(lvel, rvel, odoTh, lastEncoderTime, newEncoderTime, dt);
        odoX = odoX + dOdoX;
        odoY = odoY + dOdoY;
        odoTh = odoTh + dOdoTh;
        lastEncoderX = newEncoderX;
        lastEncoderY = newEncoderY;
        lastEncoderTime = newEncoderTime;
        %disp([odoX, odoY, odoTh]);
        
    end
    pose(length(pose) + 1, :) = [odoX, odoY, odoTh];
    plot(pose(:, 1), pose(:,2));
    pause(0.01);
    
end
disp([odoX, odoY, odoTh]);

tic;
while toc < 0.2
    robIF.sendVelocity(0, 0);
    pause(0.01);
end
hold off


%clear