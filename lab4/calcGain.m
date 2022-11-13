function [gain] = calcGain(e, de, ie, p, i, d)
%CALCGAIN Summary of this function goes here
%   Detailed explanation goes here
    gain = e*p + de * d + ie * i;
end

