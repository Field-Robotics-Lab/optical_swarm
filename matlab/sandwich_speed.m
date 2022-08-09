function [fwd] = sandwich_speed(speed)
% sandwich_speed performs an OL lookup for forward thrust given a desired
% speed. Look-up table data comes from a 4th-order fit of experimental
% speed test data

q = [-9.530184048425071e-07,4.447147138009057e-05,0.003182336729309,0.007071998994770,3.910587170558266e-05];
fwd = polyval(q,speed);

end