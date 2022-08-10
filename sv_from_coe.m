function [R_ECI, V_ECI] = sv_from_coe(Orbit)
%rp = (h^2/mu) * (1/(1 + e*cos(TA))) * (cos(TA)*[1;0;0] + sin(TA)*[0;1;0]);
%vp = (mu/h) * (-sin(TA)*[1;0;0] + (e + cos(TA))*[0;1;0]);

a=Orbit.a;                     %Semi Major Axis (km)
incl=Orbit.incl*pi/180;        %inclination (rad)
RA=Orbit.RAAN*pi/180;          %Right Ascension (rad)
e=Orbit.e;                     %Eccentricity       
w=Orbit.omega*pi/180;          %Argument of Perigee (rad)
TA=Orbit.TA*pi/180;            %True Anomaly (rad)

mu = 3.986004418e5;
p=a*(1-e^2);     %semiparameter
h=sqrt(mu*p);    %specific angular momentum

R_pqw=[p*cos(TA)/(1+e*cos(TA)); p*sin(TA)/(1+e*cos(TA)); 0];

% rdot=sqrt(mu/p)*e*sin(TA);
% rvdot=sqrt(mu/p)*(1+e*cos(TA));
% vp=[rdot*cos(TA)-rvdot*sin(TA) ; rdot*sin(TA)+rvdot*cos(TA); 0];

V_pqw=[-sqrt(mu/p)*sin(TA); sqrt(mu/p)*(e+cos(TA)) ;0];
R3_RA = [ cos(RA)  -sin(RA)   0
          sin(RA)   cos(RA)   0
            0          0      1];
        
R1_i = [1      0         0
        0   cos(incl) -sin(incl)
        0   sin(incl) cos(incl)];

    
R3_w = [ cos(w) -sin(w) 0
        sin(w) cos(w) 0
           0      0    1];
       
Q_pX = R3_RA*R1_i*R3_w;

R_ECI = Q_pX*R_pqw; %Km
V_ECI = Q_pX*V_pqw; %Km/s

%[r_ijk,v_ijk] = keplerian2ijk(9.1228e6,0.1,120,100,25,30)
end
