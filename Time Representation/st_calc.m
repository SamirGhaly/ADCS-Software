function th_GMST=st_calc(JD)
T_UT1= (JD-2451545)/36525;
th_GMST_s=67310.54841+(876600*3600+8640184.812866)*T_UT1+0.0093104*T_UT1^2-6.2e-6*T_UT1^3; % [s]
th_scaled=(th_GMST_s/86400-floor(th_GMST_s/86400));

th_GMST= 2*pi*th_scaled; % [rad]
th_GMST_deg= th_GMST*180/pi;    % [degrees]
end