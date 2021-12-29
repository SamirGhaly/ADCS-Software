function [V_F]=ECEF2ECI(V_F,utc) %Earth Fixed to Earth Inertial
t=JDate(utc);
a=st_calc(t);

R_IF=[cos(a) sin(a) 0
    -sin(a)  cos(a) 0
       0       0    1];
V_I=R_IF^-1*V_F';
end