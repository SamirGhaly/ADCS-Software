function [state_transition,F]=state_trans(Inertia,p,q,r,w_O,q1,q2,q3,q4,timestep)
Ixx=Inertia(1,1);
Ixy=Inertia(1,2);
Ixz=Inertia(1,3);
Iyx=Inertia(2,1);
Iyy=Inertia(2,2);
Iyz=Inertia(2,3);
Izx=Inertia(3,1);
Izy=Inertia(3,2);
Izz=Inertia(3,3);
F=[                   0,              -conj(p)/2, conj(w_O)/2 - conj(q)/2,                -conj(r)/2,                                                                                                                                                                                                                                                                                                                                                                                                         -conj(q2)/2,                                                                                                                                                                                                                                                                                                                                                                                                          -conj(q3)/2,                                                                                                                                                                                                                                                                                                                                                                                                         -conj(q4)/2;
              conj(p)/2,                       0,               conj(r)/2, - conj(q)/2 - conj(w_O)/2,                                                                                                                                                                                                                                                                                                                                                                                                          conj(q1)/2,                                                                                                                                                                                                                                                                                                                                                                                                          -conj(q4)/2,                                                                                                                                                                                                                                                                                                                                                                                                          conj(q3)/2;
conj(q)/2 - conj(w_O)/2,              -conj(r)/2,                       0,                 conj(p)/2,                                                                                                                                                                                                                                                                                                                                                                                                          conj(q4)/2,                                                                                                                                                                                                                                                                                                                                                                                                           conj(q1)/2,                                                                                                                                                                                                                                                                                                                                                                                                         -conj(q2)/2;
             conj(r)/2, conj(q)/2 + conj(w_O)/2,              -conj(p)/2,                         0,                                                                                                                                                                                                                                                                                                                                                                                                         -conj(q3)/2,                                                                                                                                                                                                                                                                                                                                                                                                           conj(q2)/2,                                                                                                                                                                                                                                                                                                                                                                                                          conj(q1)/2;
                     0,                       0,                       0,                         0, (Ixz*Iyy^2*q + Ixz*Izy^2*q - Ixy*Iyz^2*r - Ixy*Izz^2*r - 2*Ixy*Iyx*Iyz*p + 2*Ixz*Iyx*Iyy*p - 2*Ixy*Izx*Izz*p + 2*Ixz*Izx*Izy*p + Ixx*Ixy*Iyz*q - Ixx*Ixz*Iyy*q - Ixy*Iyy*Iyz*q - Ixy*Izy*Izz*q - Iyy*Izx*Izz*q + Iyz*Izx*Izy*q + Ixx*Ixy*Izz*r - Ixx*Ixz*Izy*r + Ixz*Iyy*Iyz*r + Iyx*Iyy*Izz*r - Iyx*Iyz*Izy*r + Ixz*Izy*Izz*r)/(Ixx*Iyy*Izz - Ixx*Iyz*Izy - Ixy*Iyx*Izz + Ixy*Iyz*Izx + Ixz*Iyx*Izy - Ixz*Iyy*Izx),          (r*Ixy^2*Izz + 2*Iyz*q*Ixy^2 - 2*q*Ixy*Ixz*Iyy - r*Ixy*Ixz*Izy + Iyz*r*Ixy*Ixz - Iyz*p*Ixy*Iyy - p*Ixy*Izy*Izz + Ixx*Iyz*p*Ixy - r*Ixz^2*Iyy + p*Ixz*Iyy^2 - Ixx*p*Ixz*Iyy + p*Ixz*Izy^2 + r*Iyy^2*Izz - 2*q*Iyy*Izy*Izz - Iyz*r*Iyy*Izy - r*Iyy*Izz^2 - Izx*p*Iyy*Izz + 2*Iyz*q*Izy^2 + Iyz*r*Izy*Izz + Iyz*Izx*p*Izy)/(Ixx*Iyy*Izz - Ixx*Iyz*Izy - Ixy*Iyx*Izz + Ixy*Iyz*Izx + Ixz*Iyx*Izy - Ixz*Iyy*Izx),      -(- q*Ixy^2*Izz - q*Ixy*Ixz*Iyz - 2*r*Ixy*Ixz*Izz + Izy*q*Ixy*Ixz + p*Ixy*Iyz^2 + p*Ixy*Izz^2 - Ixx*p*Ixy*Izz + q*Ixz^2*Iyy + 2*Izy*r*Ixz^2 - p*Ixz*Iyy*Iyz - Izy*p*Ixz*Izz + Ixx*Izy*p*Ixz - q*Iyy^2*Izz - 2*r*Iyy*Iyz*Izz + Izy*q*Iyy*Iyz + q*Iyy*Izz^2 - Iyx*p*Iyy*Izz + 2*Izy*r*Iyz^2 - Izy*q*Iyz*Izz + Iyx*Izy*p*Iyz)/(Ixx*Iyy*Izz - Ixx*Iyz*Izy - Ixy*Iyx*Izz + Ixy*Iyz*Izx + Ixz*Iyx*Izy - Ixz*Iyy*Izx);
                      0,                       0,                       0,                         0,        -(q*Ixx^2*Iyz + r*Ixx^2*Izz - 2*p*Ixx*Iyx*Iyz - Ixz*q*Ixx*Iyx - r*Ixx*Iyz^2 - Iyy*q*Ixx*Iyz - 2*p*Ixx*Izx*Izz - Ixz*r*Ixx*Izx - r*Ixx*Izz^2 - Izy*q*Ixx*Izz + r*Iyx^2*Izz + 2*Ixz*p*Iyx^2 - r*Iyx*Iyz*Izx + Ixz*r*Iyx*Iyz - q*Iyx*Izx*Izz + Ixz*Iyy*q*Iyx + q*Iyz*Izx^2 + 2*Ixz*p*Izx^2 + Ixz*r*Izx*Izz + Ixz*Izy*q*Izx)/(Ixx*Iyy*Izz - Ixx*Iyz*Izy - Ixy*Iyx*Izz + Ixy*Iyz*Izx + Ixz*Iyx*Izy - Ixz*Iyy*Izx), -(Ixx^2*Iyz*p + Iyz*Izx^2*p - Ixz^2*Iyx*r - Iyx*Izz^2*r - Ixx*Ixz*Iyx*p - Ixx*Iyy*Iyz*p + Ixz*Iyx*Iyy*p - Ixx*Izy*Izz*p + Ixz*Izx*Izy*p - Iyx*Izx*Izz*p + 2*Ixx*Ixy*Iyz*q - 2*Ixy*Ixz*Iyx*q - 2*Iyx*Izy*Izz*q + 2*Iyz*Izx*Izy*q + Ixx*Ixz*Iyz*r + Ixx*Ixy*Izz*r - Ixy*Ixz*Izx*r + Iyx*Iyy*Izz*r - Iyy*Iyz*Izx*r + Iyz*Izx*Izz*r)/(Ixx*Iyy*Izz - Ixx*Iyz*Izy - Ixy*Iyx*Izz + Ixy*Iyz*Izx + Ixz*Iyx*Izy - Ixz*Iyy*Izx),       (- p*Ixx^2*Izz - q*Ixx*Ixz*Iyz - 2*r*Ixx*Ixz*Izz + Izx*p*Ixx*Ixz + p*Ixx*Iyz^2 + p*Ixx*Izz^2 - Ixy*q*Ixx*Izz + q*Ixz^2*Iyx + 2*Izx*r*Ixz^2 - p*Ixz*Iyx*Iyz - Izx*p*Ixz*Izz + Ixy*Izx*q*Ixz - p*Iyx^2*Izz - 2*r*Iyx*Iyz*Izz + Izx*p*Iyx*Iyz + q*Iyx*Izz^2 - Iyy*q*Iyx*Izz + 2*Izx*r*Iyz^2 - Izx*q*Iyz*Izz + Iyy*Izx*q*Iyz)/(Ixx*Iyy*Izz - Ixx*Iyz*Izy - Ixy*Iyx*Izz + Ixy*Iyz*Izx + Ixz*Iyx*Izy - Ixz*Iyy*Izx);
                      0,                       0,                       0,                         0,         (q*Ixx^2*Iyy + r*Ixx^2*Izy - 2*p*Ixx*Iyx*Iyy - Ixy*q*Ixx*Iyx - q*Ixx*Iyy^2 - Iyz*r*Ixx*Iyy - 2*p*Ixx*Izx*Izy - Ixy*r*Ixx*Izx - q*Ixx*Izy^2 - Izz*r*Ixx*Izy + r*Iyx^2*Izy + 2*Ixy*p*Iyx^2 - r*Iyx*Iyy*Izx + Ixy*q*Iyx*Iyy - q*Iyx*Izx*Izy + Ixy*Iyz*r*Iyx + q*Iyy*Izx^2 + 2*Ixy*p*Izx^2 + Ixy*q*Izx*Izy + Ixy*Izz*r*Izx)/(Ixx*Iyy*Izz - Ixx*Iyz*Izy - Ixy*Iyx*Izz + Ixy*Iyz*Izx + Ixz*Iyx*Izy - Ixz*Iyy*Izx),       -(- p*Ixx^2*Iyy - 2*q*Ixx*Ixy*Iyy - r*Ixx*Ixy*Izy + Iyx*p*Ixx*Ixy + p*Ixx*Iyy^2 - Ixz*r*Ixx*Iyy + p*Ixx*Izy^2 + r*Ixy^2*Izx + 2*Iyx*q*Ixy^2 - Iyx*p*Ixy*Iyy - p*Ixy*Izx*Izy + Ixz*Iyx*r*Ixy + r*Iyy^2*Izx - p*Iyy*Izx^2 - 2*q*Iyy*Izx*Izy - Izz*r*Iyy*Izx - Iyx*r*Iyy*Izy + Iyx*p*Izx*Izy + 2*Iyx*q*Izy^2 + Iyx*Izz*r*Izy)/(Ixx*Iyy*Izz - Ixx*Iyz*Izy - Ixy*Iyx*Izz + Ixy*Iyz*Izx + Ixz*Iyx*Izy - Ixz*Iyy*Izx), (Ixx^2*Izy*p + Iyx^2*Izy*p - Ixy^2*Izx*q - Iyy^2*Izx*q - Ixx*Ixy*Izx*p - Ixx*Iyy*Iyz*p + Ixy*Iyx*Iyz*p - Iyx*Iyy*Izx*p - Ixx*Izy*Izz*p + Ixy*Izx*Izz*p + Ixx*Ixz*Iyy*q - Ixy*Ixz*Iyx*q + Ixx*Ixy*Izy*q + Iyx*Iyy*Izy*q - Iyx*Izy*Izz*q + Iyy*Izx*Izz*q + 2*Ixx*Ixz*Izy*r - 2*Ixy*Ixz*Izx*r + 2*Iyx*Iyz*Izy*r - 2*Iyy*Iyz*Izx*r)/(Ixx*Iyy*Izz - Ixx*Iyz*Izy - Ixy*Iyx*Izz + Ixy*Iyz*Izx + Ixz*Iyx*Izy - Ixz*Iyy*Izx)];

state_transition=eye(7)+F*timestep+((F*timestep)^2)/2+((F*timestep)^3)/(3*2);
end