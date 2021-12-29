%this function is to convert the civil date to the Julian date
%hours are given in the form of 24-hour time

function JD= JDate(UTC)
Y=UTC(1) ;%year
M=UTC(2) ;%month
D=UTC(3) ;%day
H=UTC(4) ;%hour
Min=UTC(5) ;%minutes
Sec=UTC(6) ;%seconds
JD= 367*Y-floor(7*(Y+round((M+9)/12))/4)+floor(275*M/9)+D+1721013.5+(((Sec/60)+Min)/60+H)/24;
%MJD = JD - 2400000.5;
end



    
    


