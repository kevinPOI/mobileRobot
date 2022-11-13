global encoderFrame;
global encoderDataReady;
global encoderDataTimeStamp;
global encoderDataStarted;
global encoderDataTimeStart;

usePid = true;
encoderDataStarted = false;
tic;
robIF = robotIF(0, true);
robIF.encoders.NewMessageFcn = @encoderEventListener;
pause(0.1);
lEncZero = robIF.encoders.LatestMessage.Vector.X;
rEncZero = robIF.encoders.LatestMessage.Vector.Y;

lastEncoderTime = encoderDataTimeStamp;
startEncoderTime = lastEncoderTime;

target = 0;
origin = (lEncZero + rEncZero) / 2;
state = (robIF.encoders.LatestMessage.Vector.X + robIF.encoders.LatestMessage.Vector.Y) / 2 - origin;
e = target - state;
lastE = e;
ie = 1;
record = [0, encoderDataTimeStamp - startEncoderTime, 0, encoderDataTimeStamp - startEncoderTime];

stopping = false;

refDist = 0;
lastRefDist = 0;
beginRobTime = robIF.toc();
lastRobTime = 0;
while(toc < 8)
    if encoderDataTimeStamp == lastEncoderTime
        pause(0.02);
        continue
    end
    robTime = robIF.toc() - beginRobTime;
    state = (robIF.encoders.LatestMessage.Vector.X + robIF.encoders.LatestMessage.Vector.Y) / 2 - origin;
    e = target - state;
    de = e - lastE;
    dt = (encoderDataTimeStamp - lastEncoderTime);
    robDt = robTime - lastRobTime;
    ie = ie + dt * e;
    
    lastE = e;
    lastRobTime = robTime;
    lastEncoderTime = encoderDataTimeStamp;
    t = lastEncoderTime;
    gain = calcGain(e, de, ie, 18, 0, 6);
%     if gain < 0.01 && stopping == false && usePid == true && false
%        tic;
%        disp("stopping")
%        stopping = true;
%     end
%     if (stopping == true && toc > 1.5)
%         break;
%     end
%     
    
    vel = trapVel(t, 0.75, 0.25, 1,1);
    lastRefDist = refDist;
    refDist = refDist + vel * robDt;
    if refDist > 0.95 && vel == 0
        refDist = 1;
    end
    if usePid
        vel = vel + gain * 0.2;
    end
    
    if vel > 0.25
        vel = 0.25;
    end
    if vel < -0.25
        vel = -0.25;
    end
    
    target = lastRefDist;
    sendVelocity(robIF, vel, vel);
    record(length(record(:,1)) + 1,:) = [state, encoderDataTimeStamp - startEncoderTime, refDist, robTime];
    %disp([e,de,ie])
    pause(0.01);
    
end
for i = 1 : length(record(:,4))
    record(i,4) = record(i,4) + 0.2;
    if record(i, 4) < 0
        record(i, 4) = 0;
    end
end
hold on
plot(record(:,2), record(:,1));
plot(record(:,4), record(:,3));
disp(refDist)
tic;
while(toc < 2)
    sendVelocity(robIF, 0, 0);
end
