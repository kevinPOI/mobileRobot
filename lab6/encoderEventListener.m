function encoderEventListener(handle,event)
%   encoderEventListener Invoked when new encoder data arrives.
%   A MATLAB event listener for the Robot. Invoked when encoder data
%   arrives.
%   AL note June 2020 There was a long standing "bug" here where the
%   nanoseconds were not in the start time. This was probably a deliberate
%   attempt to mimic a different and varying time stanard on the robot. I
%   took it out.

global encoderFrame;
global encoderDataReady;
global encoderDataTimeStamp;
global encoderDataStarted;
global encoderDataTimeStart;

if(~encoderDataStarted)
    encoderDataStarted = true;
    %encoderDataTimeStart = double(event.Header.Stamp.Sec);          % Al: June 2020
    encoderDataTimeStart = raspbot.timeFromStamp(event.Header.Stamp); % Al: June 2020
    disp('Encoder Event Listener is Up\n');
end
encoderDataTimeStamp = raspbot.timeFromStamp(event.Header.Stamp)-encoderDataTimeStart;
%fprintf('in event listener\n');
%disp("h");
encoderFrame = encoderFrame + 1;
encoderDataReady = 1;
end


