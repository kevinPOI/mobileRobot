function vel = trapVel(t, amax,vmax, dist, sgn)
%TRAPVEL Summary of this function goes here
%   Detailed explanation goes here
tramp = vmax / amax;
dconst = dist - tramp * vmax;
tconst = dconst / vmax;
if t < tramp
    vel = t * amax;
elseif t >= tramp && t <= tramp + tconst;
    vel = vmax;
elseif t <= tramp * 2 + tconst
    vel = vmax - (t - tramp - tconst) * amax;
else
    vel = 0;
end
vel = vel * sgn;
end

