function [fwd] = sandwich_speed(speed)
% sandwich_speed performs an OL lookup for forward thrust given a desired
% speed. Look-up table data comes from a 4th-order fit of experimental
% speed test data

q = [-3.320753108855899e-07,1.441431517845468e-05,0.002194321256392,0.002222559249645,-5.238133140578684e-06];
fwd = polyval(q,speed);

end