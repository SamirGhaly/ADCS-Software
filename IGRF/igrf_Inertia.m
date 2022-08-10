function [B_ECI,latitude,longitude,rhom]=igrf_Inertia(R_ECI)

global date B_ECEF 
Date_Str=datestr(date);

R_ECEF=  ECI2ECEF(R_ECI,date);         %km
geod=ecef2geod(R_ECEF*1e3);

latitude=geod(1);
longitude=geod(2);
rhom=geod(3);

B_NED = igrf_2(Date_Str,latitude,longitude,rhom*1e-3,'geodetic');

q_E_NED=conjq(q_ned_ecf(latitude,longitude));

B_ECEF=newfrm(B_NED,q_E_NED);

[R_IF]=ECI2ECEF_D(date);  %Rotation Matrix

q_E_I=ConvertRep(R_IF,'DCM','Q');
%q_E_I_2=dcm2quat(R_IF);
q_I_E=conjq(q_E_I);
B_ECI=newfrm(B_ECEF,q_I_E);

end