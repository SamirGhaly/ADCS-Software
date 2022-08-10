function g=gravity(x,y,z,utc)
V_ECI=[x;y;z];
[V_ECEF]=ECI2ECEF(V_ECI,utc);
% r = sqrt(x.^2+y.^2+z.^2) ;
r = sqrt(V_ECEF(1).^2+V_ECEF(2).^2+V_ECEF(3).^2) ;
Mu = 3.98600441500e5;
Re =6378140e-3 ; %Km
% Cnm = zeros(5,5) ;
% Snm = zeros(5,5) ;
V_nm = zeros(5,5) ;

pU_pX = 0 ;
pU_pY = 0 ;
pU_pZ = 0 ;
%%%%%%%%%%%%%%%%%%%%%%%%%    CALCULATE  Cnm & Snm
% we assume that 1 is as the starting point instead of 0
% Cnm(1,1) = 1 ;                           Snm(1,1) = 0 ;
% Cnm(2,1) = 0 ;                           Snm(2,1) = 0 ;
% Cnm(2,2) = 0 ;                           Snm(2,2) = 0 ;
% Cnm(3,1) = -1082.637e-6 ;                Snm(3,1) = 0 ;
% Cnm(3,2) = 0 ;                           Snm(3,2) = 0 ;
% Cnm(3,3) = 1.54e-6 ;                     Snm(3,3) = -0.88e-6 ;
% Cnm(4,1) =  2.54e-6 ;                    Snm(4,1) = 0 ;
% Cnm(4,2) = 2.16e-6 ;                     Snm(4,2) = 0.24e-5 ;
% Cnm(4,3) = 0.27e-6 ;                     Snm(4,3) = 0.26e-6 ;
% Cnm(4,4) = 0.68e-7 ;                     Snm(4,4) = 0.21e-6 ;
% Cnm(5,1) = 1.62e-6 ;                     Snm(5,1) = 0 ;
% Cnm(5,2) =  -0.49e-6 ;                   Snm(5,2) = -0.46e-6 ;
% Cnm(5,3) = 0.77e-7 ;                     Snm(5,3) = 0.15e-6 ;
% Cnm(5,4) = 0.62e-7 ;                     Snm(5,4) = -0.71e-8;
% Cnm(5,5) = -0.22e-8 ;                    Snm(5,5) = 0.75e-8 ;

% Cnm(6,1) = -2.2800e-7 ;   Snm(6,1) = 0 ;
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
Cnm      = [ 1                         0                       0                       0                       0
                    0                         0                       0                       0                       0
                    -0.1082626925638815e-2    -0.2414000052222093e-9   0.1574421758350994e-5   0                       0
                    0.2532307818191774e-5     0.2190922081404716e-5   0.3089143533816488e-6   0.100561040626586e-6    0
                    0.1620429990000000e-5    -0.5088433157745930e-6   0.7834048953908266e-7   0.5917924178248455e-7  -0.3982546443559900e-8];
                
 Snm=[ 0                         0                       0                       0                       0
                    0                         0                       0                       0                       0
                    0                         0.1543099973784379e-8  -0.9037666669616874e-6   0                       0
                    0                         0.2687418863136855e-6  -0.2115075122835371e-6   0.1971780250456937e-6   0
                    0                        -0.4491281706406470e-6   0.1482219920570510e-6  -0.1201263975958658e-7   0.6525548406274755e-8];               


Cnm = double(Cnm) ;  
Snm = double(Snm) ;
for n=1:5
    N=n+1;
    for m=0:n
        M=m+1;
        % calculate V_nm 
        V_nm(1,1) = 1/r ;
        V_nm(N,N) = (2*n-1) *((x+y*1i)/(r^2)).*V_nm(N-1,N-1);
        if n-m==1
          
            V_nm(N,M)= (2*n-1)/(n-m)*(z/(r^2))*V_nm(N-1,M) ;
            
        elseif n-m>1
            
            V_nm(N,M)= ((2*n-1)/(n-m)*(z/(r^2))*V_nm(N-1,M))-((n+m-1)/(n-m)*(1/(r^2))*V_nm(N-2,M)) ;
            
        end
        
        if n==5
            for k=1:n-1
                 V_nm(N,k)= ((2*n-1)/(n-k)*(z/(r^2))*V_nm(N-1,k))-((n+k-1)/(n-k)*(1/(r^2))*V_nm(N-2,k)) ;
            end
        end
    end
end
      %% for X & Y ================================================================
        % calculate pVnm_pX
pVnm_pX=zeros(5);
pVnm_pY=zeros(5);
pVnm_pZ=zeros(5);

for n=0:4
    N=n+1;
    for m=0:n
        M=m+1;
        if n<5
            if m==0

                pVnm_pX(N,M) = -0.5 *(V_nm(N+1,2)+conj(V_nm(N+1,2))) ;
                pVnm_pY(N,M) = 0.5i*(V_nm(N+1,2)-conj(V_nm(N+1,2)));

            elseif m>0

                pVnm_pX(N,M) = -.5*(V_nm(N+1,M+1)-(factorial(n-m+2)./factorial(n-m)).*V_nm(N+1,M-1));
                pVnm_pY(N,M) = .5i*(V_nm(N+1,M+1)+(factorial(n-m+2)./factorial(n-m)).*V_nm(N+1,M-1));

            end
             pVnm_pZ(N,M) = - (factorial(n-m+1)/factorial(n-m))*V_nm(N+1,M) ;
        end
    end
end
for  n=0:4
    N=n+1;
    for m=0:n
        M=m+1;
   
       pU_pX(N,M) =((Re^n)*(Cnm(N,M)-1i*Snm(N,M))*pVnm_pX(N,M)) ;
       pU_pY(N,M) =((Re^n)*(Cnm(N,M)-1i*Snm(N,M))*pVnm_pY(N,M)) ;
       pU_pZ(N,M) =((Re^n)*(Cnm(N,M)-1i*Snm(N,M))*pVnm_pZ(N,M)) ;     
            
    end
end
%% Summation
pU_pX_total=sum(pU_pX,'all');
pU_pY_total=sum(pU_pY,'all');
pU_pZ_total=sum(pU_pZ,'all');

pU_pX_total=real(pU_pX_total);
pU_pY_total=real(pU_pY_total);
pU_pZ_total=real(pU_pZ_total);
g= Mu.*[pU_pX_total;pU_pY_total;pU_pZ_total];
end



















