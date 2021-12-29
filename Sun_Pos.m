function [Sun_Pos_Vec_AU,Sun_Pos_Vec_Km] = Sun_Pos( Julian_Date)
format long
%Julian Centuries
T_ut1= (Julian_Date-2451545)/36525;

%mean longitude of the sun
lamda_M = wrapTo360(280.46+ 36000.771*T_ut1);

%mean anomly of the sun
M= wrapTo360(357.5291092+ 35999.050*T_ut1); 

%ecliptic longitude
lamda_ecliptic= wrapTo360(lamda_M + 1.914666471*sind(M)+0.019994643*sind(2*M));

%ecliptic latitude
phai_ecliptic= 0;

%distance from earth to sun
r=1.00014612- 0.016708617*cosd(M)-0.000139589*cosd(2*M);

%obliquity of the ecliptic
epsilon= wrapTo360(23.439291-0.0130042*T_ut1);

%Calculation of The Sun Position Vector (in geocentric equatorial coordinates)
%Astronomical Unit
Sun_Pos_Vec_AU= [r*cosd(lamda_ecliptic);
              r*cosd(epsilon)*sind(lamda_ecliptic);
              r*sind(epsilon)*sind(lamda_ecliptic)];
          
%Km
Sun_Pos_Vec_Km= 149597871*[r*cosd(lamda_ecliptic);
              r*cosd(epsilon)*sind(lamda_ecliptic);
              r*sind(epsilon)*sind(lamda_ecliptic)];
          
end
    









