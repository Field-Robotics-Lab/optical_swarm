function [fwd] = sandwich_speed(speed)
% sandwich_speed performs an OL lookup for forward thrust given a desired
% speed. Look-up table data comes from a 4th-order fit of experimental
% speed test data

q = [8.787516741419185e-07,-1.797707086229711e-06,0.010252082562086,0.001525282390623,-8.217698480861404e-07];
fwd = polyval(q,speed);

end