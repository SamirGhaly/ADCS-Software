function eqn=getDCM(x,B_O,B_observed)
% syms x(1) x(2) x(3)
eqn =[cos(x(2))*cos(x(1)) -cos(x(3))*sin(x(1))+sin(x(3))*sin(x(2))*cos(x(1)) sin(x(3))*sin(x(1))+cos(x(3))*sin(x(2))*cos(x(1));
    cos(x(2))*sin(x(1))  cos(x(3))*sin(x(1))+sin(x(3))*sin(x(2))*sin(x(1)) -sin(x(3))*cos(x(1))+cos(x(3))*sin(x(2))*sin(x(1));
    -sin(x(2))   sin(x(3))*cos(x(2))  cos(x(3))*cos(x(2))]*B_O -B_observed;

% Attitude=solve(eqn,[x(1),x(2),x(3)]);

end