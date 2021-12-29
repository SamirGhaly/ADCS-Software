function [g, h] = loadigrfcoefs(time)

% LOADIGRFCOEFS Load coefficients used in IGRF model.
% 
% Usage: [G, H] = LOADIGRFCOEFS(TIME) or GH = LOADIGRFCOEFS(TIME)
% 
% Loads the coefficients used in the IGRF model at time TIME in MATLAB
% serial date number format and performs the necessary interpolation. If
% two output arguments are requested, this returns the properly
% interpolated matrices G and H from igrfcoefs.mat. If just one output is
% requested, the proper coefficient vector GH from igrfcoefs.mat is
% returned.


% Convert time to a datenumber if it is a string.
if ischar(time)
    time = datenum(time);
end


% Convert time to fractional years.
timevec = datevec(time);
time = timevec(1) + (time - datenum([timevec(1) 1 1]))./(365 + double(...
    (~mod(timevec(1),4) & mod(timevec(1),100)) | (~mod(timevec(1),400))));


load igrfcoefs.mat;

% % Check validity on time.
yrs = cell2mat({coefs.year});


% Get the nearest epoch that the current time is between.
lastepoch = find(yrs - time < 0, 1, 'last');
if isempty(lastepoch)
    lastepoch = 1;
end
nextepoch = lastepoch + 1;

% Output either g and h matrices or gh vector depending on the number of
% outputs requested.

      
% Get the coefficients based on the epoch.
lastgh = coefs(lastepoch).gh;
nextgh = coefs(nextepoch).gh;

% If one of the coefficient vectors is smaller than the other, enlarge
% the smaller one with 0's.
if length(lastgh) > length(nextgh)
    smalln = length(nextgh);
    nextgh = zeros(size(lastgh));
    nextgh(1:smalln) = coefs(nextepoch).gh;
elseif length(lastgh) < length(nextgh)
    smalln = length(lastgh);
    lastgh = zeros(size(nextgh));
    lastgh(1:smalln) = coefs(lastepoch).gh;
end

% Calculate gh using a linear interpolation between the last and next
% epoch.
if coefs(nextepoch).slope
    ghslope = nextgh;
else
    ghslope = (nextgh - lastgh)/(yrs(lastepoch)- yrs(nextepoch));
end
g = lastgh + ghslope*(time - yrs(lastepoch));

end