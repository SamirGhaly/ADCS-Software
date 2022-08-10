function V_NewFrame=newfrm(v,q)

v_old=[0 v(1) v(2) v(3)];  %As Quaternion
V=quatmul(quatmul(conjq(q),v_old),q);
V_NewFrame=[V(2) V(3) V(4)]'; %Coloumn
end