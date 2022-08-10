function [R_IF]=ECI2ECEF_D(utc) %Earth Inertial to Earth Fixed
t=JDate(utc);
a=st_calc(t);
R_IF=[cos(a) sin(a) 0
    -sin(a)  cos(a) 0
       0       0    1];
end