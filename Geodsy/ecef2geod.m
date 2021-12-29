function [latitude, longitude, altitude] = ecef2geod(x, y, z)
% Converts Earth-centered, Earth fixed (ECEF) coordinates X, Y, and Z to
% geodetic coordinates LATITUDE, LONGITUDE, and ALTITUDE.The World Geodetic System 1984
% (WGS84) ellipsoid model of the Earth is assumed.
% 
% Inputs:
% x,y,z in meters
% Ouputs:
% latitude, longitude in degrees.
% altitude in meters.


% WGS84 parameters.
a = 6378137; f = 1/298.257223563; b = a*(1 - f); e2 = 1 - (b/a)^2;
tol=1e-12; %Maximum error tolerance in the latitude in radians (optional,default is 1e-12).

longitude = atan2(y, x)*180/pi;

% Compute latitude by iterations.
rd = sqrt(x.^2 + y.^2);
[latitude, Nphi] = lat_iterations(asin(z ./ sqrt(x.^2+y.^2+z.^2)), z, a, e2, ...
    rd, tol, 1);

sinlat = sin(latitude); coslat = cos(latitude); latitude = latitude*180/pi;

% Get altitude from latitude.
altitude = rd.*coslat + (z + e2*Nphi.*sinlat).*sinlat - Nphi;
end

function [latitude, Nphi] = lat_iterations(lat_in, z, a, e2, rd, tol, iter)
thisNphi = a ./ sqrt(1 - e2*sin(lat_in).^2);
nextlat = atan((z + thisNphi*e2.*sin(lat_in))./rd);
if all(abs(lat_in - nextlat) < tol) || iter > 100
    latitude = nextlat; Nphi = thisNphi;
else
    [latitude, Nphi] = lat_iterations(nextlat, z, a, e2, rd, tol, iter + 1);
end
end