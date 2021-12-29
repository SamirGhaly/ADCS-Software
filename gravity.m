clc
clear vars
close all
format long
x= 5000;
y= 5000;
z= 6378140 ;
r = sqrt(x.^2+y.^2+z.^2) ;
Mu = 3.98600441800e14;
Re =6378140 ;
Cnm = zeros(5,5) ;
Snm = zeros(5,5) ;
V_nm = zeros(5,5) ;
pU_pX = 0 ;
pU_pY = 0 ;
pU_pZ = 0 ;
%%%%%%%%%%%%%%%%%%%%%%%%%    CALCULATE  Cnm & Snm
% we assume that 1 is as the starting point instead of 0
Cnm(1,1) = 1 ;                           Snm(1,1) = 0 ;
Cnm(2,1) = 0 ;                           Snm(2,1) = 0 ;
Cnm(2,2) = 0 ;                           Snm(2,2) = 0 ;
Cnm(3,1) = -1082.637e-6 ;                Snm(3,1) = 0 ;
Cnm(3,2) = 0 ;                           Snm(3,2) = 0 ;
Cnm(3,3) = 1.54e-6 ;                     Snm(3,3) = -0.88e-6 ;
Cnm(4,1) =  2.54e-6 ;                    Snm(4,1) = 0 ;
Cnm(4,2) = 2.16e-6 ;                     Snm(4,2) = 0.24e-5 ;
Cnm(4,3) = 0.27e-6 ;                     Snm(4,3) = 0.26e-6 ;
Cnm(4,4) = 0.68e-7 ;                     Snm(4,4) = 0.21e-6 ;
Cnm(5,1) = 1.62e-6 ;                     Snm(5,1) = 0 ;
Cnm(5,2) =  -0.49e-6 ;                   Snm(5,2) = -0.46e-6 ;
Cnm(5,3) = 0.77e-7 ;                     Snm(5,3) = 0.15e-6 ;
Cnm(5,4) = 0.62e-7 ;                     Snm(5,4) = -0.71e-8;
Cnm(5,5) = -0.22e-8 ;                    Snm(5,5) = 0.75e-8 ;
%Cnm(6,1) = -2.2800e-7 ;   Snm(6,1) = 0 ;
% Cnm(6,2) = -4.5958e-7 ;   Snm(6,2) = -6.8485e-8 ;
% Cnm(6,3) = 9.6889e-8 ;   Snm(6,3) = 6.4588e-8 ;
% Cnm(6,4) = -1.9302e-8 ;   Snm(6,4) = -5.3972e-9 ;
% Cnm(6,5) = -9.0188e-10 ;   Snm(6,5) = -3.5344e-10 ;
% Cnm(6,6) = 3.4363e-10 ;   Snm(6,6) = -2.1382e-9;
% Cnm(7,1) = 5.5201e-7 ;   Snm(7,1) = 0 ;
% Cnm(7,2) = -5.8780e-8 ;   Snm(7,2) = 1.3970e-8 ;
% Cnm(7,3) = 3.0690e-9 ;   Snm(7,3) = -5.0575e-7 ;
% Cnm(7,4) = 9.1517e-11 ;   Snm(7,4) = 6.0242e-10 ;
% Cnm(7,5) = -3.7866e-10 ;   Snm(7,5) = -1.1469e-9 ;
% Cnm(7,6) = -1.0899e-10 ;   Snm(7,6) = -4.9202e-10 ;
% Cnm(7,7) = -6.7881e-12 ;   Snm(7,7) = -6.1337e-11 ;

Cnm = double(Cnm)   ;
Snm = double(Snm) ;
for n=2:5
    for m=1:n
        
        % calculate V_nm 
        V_nm(1,1) = 1/r ;
        V_nm(n,n) = (2*n-1) *((x+y*i)/(r^2)).*V_nm(n-1,n-1);
        if n-m==1
          
            V_nm(n,m)= (2*n-1)/(n-m)*(z/(r^2))*V_nm(n-1,m) ;
            
        elseif n-m>1
            
            V_nm(n,m)= ((2*n-1)/(n-m)*(z/(r^2))*V_nm(n-1,m))-((n+m-1)/(n-m)*(1/(r^2))*V_nm(n-2,m)) ;
            
        end
        
        if n==5
            for k=1:n-1
                 V_nm(n,k)= ((2*n-1)/(n-k)*(z/(r^2))*V_nm(n-1,k))-((n+k-1)/(n-k)*(1/(r^2))*V_nm(n-2,k)) ;
            end
        end
    end
end
      %% for X & Y ================================================================
        % calculate pVnm_pX
for n=2:5
    for m=1:n
        if n<5
            if m==1

                pVnm_pX = -0.5 *(V_nm(n+1,2)+conj(V_nm(n+1,2))) ;
                pVnm_pY = -0.5i*(V_nm(n+1,2)-conj(V_nm(n+1,2)));

            elseif m>1

                pVnm_pX = -.5*(V_nm(n+1,m+1)-(factorial(n-m+2)./factorial(n-m)).*V_nm(n+1,m-1))
                pVnm_pY = .5i*(V_nm(n+1,m+1)+(factorial(n-m+2)./factorial(n-m)).*V_nm(n+1,m-1))

            end
             pVnm_pZ = - (factorial(n-m+1)/factorial(n-m))*V_nm(n+1,m) ;
     end
    %% for Z ================================================================

    %%
    

       pU_pX = real(pU_pX) +  real(((Re^n)*(Cnm(n,m)-i*Snm(n,m))*pVnm_pX)) ;
       pU_pY = real(pU_pX) +  real(((Re^n)*(Cnm(n,m)-i*Snm(n,m))*pVnm_pY)) ;
       pU_pZ = real(pU_pX) +  real(((Re^n)*(Cnm(n,m)-i*Snm(n,m))*pVnm_pZ)) ;

            
            
    end
end
    

    g  = Mu.* [pU_pX,pU_pY,pU_pZ]



















