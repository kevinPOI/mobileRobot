function [dx, dy, dth, t] = modelDiffSteerRobot(vl, vr, th, t0, tf, dt)

th = th + dt * (vr-vl)/robotModel.W / 2;
dx = (vl + vr)/2 * cos(th) * dt;
dy = (vl + vr)/2 * sin(th) * dt;
dth = dt * (vr-vl)/robotModel.W;
t = tf;
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

end