%assume we have W,w,i,v,e,U,truelong,W_true
function [ R,V ] = COE2RV(a, e, i, w, W, v)
%W=8.8057;
%w=177.13;
%i=98.085;
%v=90;
%e=0.001;
%a=7046.14;
u=3.986*10^5;
p=a*(1-e^2);
if ((norm(e)==0)&& i==0)
    w=0;
    W=0;
    v=truelong;
elseif(norm(e)==0)
    w=0;
    v=U;
    
elseif (i==0)
    W=0;
    w=W_true;
end
A=[cosd(W)*cosd(w)-sind(W)*sind(w)*cosd(i) -cosd(W)*sind(w)-sind(W)*cosd(w)*cosd(i) sind(W)*sin(i);sind(W)*cosd(w)+cosd(W)*sind(w)*cosd(i) -sind(W)*sind(w)+cosd(W)*cosd(w)*cosd(i) -cosd(W)*sin(i);sind(w)*sind(i) cosd(w)*sind(i) cosd(i)];

R=(1/(1+norm(e)*cosd(v)))*[p*cosd(v) p*sind(v) 0];
V=sqrt(u/p)*[-sind(v) (norm(e)+cosd(v)) 0];
R=A*R';
V=A*V';