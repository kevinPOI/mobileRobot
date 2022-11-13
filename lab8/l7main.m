global encoderFrame;
global encoderDataReady;
global encoderDataTimeStamp;
global encoderDataStarted;
global encoderDataTimeStart;

hold on
mrpl = mrplSystem();
mrpl.goToPoint(0.2, 0,0,0,0.3048,0.3048,0,1);
mrpl.plotRecord(0, 0, 0);
mrpl.getOdoPosition();%get current odo position

mrpl.goToPoint(0.2, mrpl.x,mrpl.y,mrpl.th,-0.3048,-0.3048,-pi/2.0,1);
mrpl.plotRecord(mrpl.x, mrpl.y, mrpl.th);
mrpl.getOdoPosition();

mrpl.goToPoint(0.2, mrpl.x,mrpl.y,mrpl.th,0,0,0,1);
mrpl.plotRecord(mrpl.x, mrpl.y, mrpl.th);
mrpl.getOdoPosition();
hold off