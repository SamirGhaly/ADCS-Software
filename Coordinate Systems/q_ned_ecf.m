function q_ned_ef=q_ned_ecf(latitude,longitude)

q_ned_ef=[cosd(longitude/2)*cosd(45+latitude/2);
    sind(longitude/2)*sind(45+latitude/2);
    -cosd(longitude/2)*sind(45+latitude/2);
    sind(longitude/2)*cosd(45+latitude/2)];
end
