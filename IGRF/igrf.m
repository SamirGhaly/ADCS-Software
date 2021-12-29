function [Bx, By, Bz] = igrf(time, latitude, longitude, altitude)
% time input for Example = 29-Dec-2021
% longitude & latitude are in degrees 
% altitude in km
% Fundamental constant.
Rearth_km = 6371.2;

%%% CHECK INPUT VALIDITY %%%
% Convert time to a datenumber if it is a string.
if ischar(time)
    time = datenum(time);
end


%%% SPHERICAL COORDINATE CONVERSION %%%
% Convert the latitude, longitude, and altitude coordinates input into
% spherical coordinates r (radius), theta (inclination angle from +z axis),
% and phi (azimuth angle from +x axis).
% We want cos(theta) and sin(theta) rather than theta itself.
costheta = cos((90 - latitude)*pi/180);
sintheta = sin((90 - latitude)*pi/180);


a = 6378.137; f = 1/298.257223563; b = a*(1 - f);
rho = sqrt(abs(a*sintheta).^2 + abs(b*costheta).^2);
r = sqrt( altitude.^2 + 2*altitude.*rho + ...
    (a^4*sintheta.^2 + b^4*costheta.^2) ./ rho.^2 );
cd = (altitude + rho) ./ r;
sd = (a^2 - b^2) ./ rho .* costheta.*sintheta./r;
oldcos = costheta;
costheta = costheta.*cd - sintheta.*sd;
sintheta = sintheta.*cd + oldcos.*sd;


% Convert longitude to radians.
phi = longitude*pi/180;

%%% GET PROPER IGRF COEFFICIENTS %%%
if isscalar(time)
    gh = loadigrfcoefs(time);
    nmax = sqrt(length(gh) + 1) - 1;
% Assume a vector input means the coefficients are the input.
else
    gh = time;
    nmax = sqrt(length(gh) + 1) - 1;
    % nmax should be an integer.
    if nmax - round(nmax) ~= 0
        error('igrf:timeInputInvalid', ['TIME input should either be ' ...
            'a single date or a valid coefficient vector.']);
    end
end

% We need cos(m*phi) and sin(m*phi) multiple times, so precalculate into a
% vector here:
cosphi = cos((1:nmax)*phi);
sinphi = sin((1:nmax)*phi);

Pmax = (nmax+1)*(nmax+2)/2;

%%% BEGIN MAGNETIC FIELD CALCULATION %%%
% Initialize variables used in for loop below.
Br = 0; Bt = 0; Bp = 0;
 P = zeros(1, Pmax);  P(1) = 1;  P(3) = sintheta;
dP = zeros(1, Pmax); dP(1) = 0; dP(3) = costheta;

% For this initial condition, the first if will result in n = 1, m = 0.
m = 1; n = 0; coefindex = 1;

a_r = (Rearth_km/r)^2;

% Increment through all the n's and m's. gh will be a vector with g
% followed by h for incrementing through n and m except when h would be
% redundant (i.e., when m = 0).
for Pindex = 2:Pmax
    
    % Increment to the next n when m becomes larger than n.
    if n < m
        m = 0;
        n = n + 1;
        a_r = a_r*(Rearth_km/r); % We need (Rearth_km./r)^(n+2)
    end
    
    if m < n && Pindex ~= 3 % (Pindex=3 is n=1, m=1, initial cond. above)
        last1n = Pindex - n;
        last2n = Pindex - 2*n + 1;
        P(Pindex) = (2*n - 1)/sqrt(n^2 - m^2)*costheta*P(last1n) - ...
            sqrt(((n-1)^2 - m^2) / (n^2 - m^2)) * P(last2n);
        dP(Pindex) = (2*n - 1)/sqrt(n^2 - m^2)*(costheta*dP(last1n) - ...
            sintheta*P(last1n)) - sqrt(((n-1)^2 - m^2) / (n^2 - m^2)) * ...
            dP(last2n);
    elseif Pindex ~= 3
        lastn = Pindex - n - 1;
        P(Pindex) = sqrt(1 - 1/(2*m))*sintheta*P(lastn);
        dP(Pindex) = sqrt(1 - 1/(2*m))*(sintheta*dP(lastn) + ...
            costheta*P(lastn));
    end
    

    if m == 0 % Implies h = 0, so only coefficient in gh is g
        coef = a_r*gh(coefindex); %*cos(0*phi) = 1
        Br = Br + (n+1)*coef*P(Pindex);
        Bt = Bt - coef*dP(Pindex);
        % Bp is 0 for m = 0.
        coefindex = coefindex + 1; % Only need to skip over g this time.
    else
        coef = a_r*(gh(coefindex)*cosphi(m) + gh(coefindex+1)*sinphi(m));
        Br = Br + (n+1)*coef*P(Pindex);
        Bt = Bt - coef*dP(Pindex);
        if sintheta == 0 % Use different formula when dividing by 0.
            Bp = Bp - costheta*a_r*(-gh(coefindex)*sinphi(m) + ...
                gh(coefindex+1)*cosphi(m))*dP(Pindex);
        else
            Bp = Bp - 1/sintheta*a_r*m*(-gh(coefindex)*sinphi(m) + ...
                gh(coefindex+1)*cosphi(m))*P(Pindex);
        
        end
        coefindex = coefindex + 2; % Skip over g and h this time.
    end
    
    % Increment m.
    m = m + 1;
    
end

% Convert from spherical to (x,y,z) = (North,East,Down).
Bx = -Bt;
By = Bp;
Bz = -Br;

% Convert back to geodetic coordinates if necessary.
Bx_old = Bx;
Bx = Bx.*cd + Bz.*sd;
Bz = Bz.*cd - Bx_old.*sd;