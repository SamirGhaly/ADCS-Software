function [OUTPUT] = ConvertRep(INPUT,input_type,output_type)
%% Rotation Matrix
ROT1 = @(t) [1   0 0;
    0   cosd(t) sind(t);
    0  -sind(t) cosd(t)];
ROT2 = @(t) [cosd(t) 0 -sind(t);
    0      1  0;
    sind(t) 0  cosd(t)];
ROT3 = @(t) [cosd(t) sind(t) 0;
    -sind(t) cosd(t) 0;
    0         0   1];

tilde_w = @(w) [0    -w(3)  w(2);
              w(3)  0    -w(1);
             -w(2)  w(1)  0];
%% DCM
if strcmpi('DCM',input_type)
%if input_type=='DCM'
    C = INPUT;
    if output_type=='E'
        OUTPUT= [atan2d(C(1,2),C(1,1));-asind(C(1,3));atan2d(C(2,3),C(3,3))];

    elseif output_type=='Q'
        OUTPUT=dcm2quaternion(INPUT);
        
    elseif output_type=='MRP'
        Q=dcm2quaternion(INPUT);
        S=NaN(3,1);
        S(1)= Q(2)/(1+Q(1));
        S(2)= Q(3)/(1+Q(1));
        S(3)= Q(4)/(1+Q(1));
        OUTPUT=S;
    end
end
%% EULER
if input_type=='E'
     eul = INPUT; %Input is row
   if output_type=='DCM'
        R = zeros(3,3,size(eul,1),'like',eul);
ct = cos(eul);
st = sin(eul);

%The default conversion in ZYX sequece

    R(1,1,:) = ct(:,2).*ct(:,1);
    R(1,2,:) = st(:,3).*st(:,2).*ct(:,1) - ct(:,3).*st(:,1);
    R(1,3,:) = ct(:,3).*st(:,2).*ct(:,1) + st(:,3).*st(:,1);
    R(2,1,:) = ct(:,2).*st(:,1);
    R(2,2,:) = st(:,3).*st(:,2).*st(:,1) + ct(:,3).*ct(:,1);
    R(2,3,:) = ct(:,3).*st(:,2).*st(:,1) - st(:,3).*ct(:,1);
    R(3,1,:) = -st(:,2);
    R(3,2,:) = st(:,3).*ct(:,2);
    R(3,3,:) = ct(:,3).*ct(:,2);
   
        OUTPUT = R
    elseif output_type=='Q'

        q = zeros(size(eul,1), 4, 'like', eul);

     % Compute sines and cosines of half angles
        c = cos(eul/2);
        s = sin(eul/2);


    % Construct quaternion
        q = [c(:,1).*c(:,2).*c(:,3)+s(:,1).*s(:,2).*s(:,3), ...
        c(:,1).*c(:,2).*s(:,3)-s(:,1).*s(:,2).*c(:,3), ...
        c(:,1).*s(:,2).*c(:,3)+s(:,1).*c(:,2).*s(:,3), ...
        s(:,1).*c(:,2).*c(:,3)-c(:,1).*s(:,2).*s(:,3)];

       OUTPUT =q;  %The default rotation sequence is 'ZYX'


    elseif output_type=='MRP'
        x=INPUT;
        C = ROT1(x(3))*ROT2(x(2))*ROT3(x(1));
        Q=dcm2quaternion(C);
        S=NaN(3,1);
        S(1)= Q(2)/(1+Q(1));
        S(2)= Q(3)/(1+Q(1));
        S(3)= Q(4)/(1+Q(1));
        OUTPUT=S; 
end
    end
        
%% Quaternion
%if strcmpi('Q',input_type)
if input_type=='Q'
    q=INPUT;
    qw = q(:,1);
    qx = q(:,2);
    qy = q(:,3);
    qz = q(:,4);
    if output_type=='DCM'
        N=sqrt(q(:,1).^2 + q(:,2).^2 + q(:,3).^2 + q(:,4).^2);
        qin=q./N;
        DCM= zeros(3,3);
        DCM(1,1,:) = qin(:,1).^2 + qin(:,2).^2 - qin(:,3).^2 - qin(:,4).^2;
        DCM(1,2,:) = 2.*(qin(:,2).*qin(:,3) + qin(:,1).*qin(:,4));
        DCM(1,3,:) = 2.*(qin(:,2).*qin(:,4) - qin(:,1).*qin(:,3));
        DCM(2,1,:) = 2.*(qin(:,2).*qin(:,3) - qin(:,1).*qin(:,4));
        DCM(2,2,:) = qin(:,1).^2 - qin(:,2).^2 + qin(:,3).^2 - qin(:,4).^2;
        DCM(2,3,:) = 2.*(qin(:,3).*qin(:,4) + qin(:,1).*qin(:,2));
        DCM(3,1,:) = 2.*(qin(:,2).*qin(:,4) + qin(:,1).*qin(:,3));
        DCM(3,2,:) = 2.*(qin(:,3).*qin(:,4) - qin(:,1).*qin(:,2));
        DCM(3,3,:) = qin(:,1).^2 - qin(:,2).^2 - qin(:,3).^2 + qin(:,4).^2;
        OUTPUT=DCM;
        
    elseif output_type=='E'  %ZYX
        aSinInput = -2*(qx.*qz-qw.*qy);
        aSinInput(aSinInput > 1) = 1;
        aSinInput(aSinInput < -1) = -1;
        
        eul = [ atan2d( 2*(qx.*qy+qw.*qz), qw.^2 + qx.^2 - qy.^2 - qz.^2 ), ...
                asind( aSinInput ), ...
                atan2d( 2*(qy.*qz+qw.*qx), qw.^2 - qx.^2 - qy.^2 + qz.^2 )];
            if ~isreal(eul) 
                eul = real(eul);
            end
       OUTPUT=eul;
       
    elseif output_type=='MRP'
        N=sqrt(q(:,1).^2 + q(:,2).^2 + q(:,3).^2 + q(:,4).^2);
        qin=q./N;
        MRP=zeros(1,3);
        for k=1:1
            MRP(k,:) = qin(k,2:4)/(1+qin(1,1));
        end
        OUTPUT=MRP;
    end
end

%% MRP
if strcmpi('MRP',input_type)
%if input_type=='MRP'
    x=INPUT;
    C = eye(3)+(8*tilde_w(x)^2-4*(1-x'*x)*tilde_w(x))/(1+x'*x)^2;
    if output_type=='DCM'
        OUTPUT=C;
    elseif output_type=='E'
         OUTPUT= [atan2d(C(1,2),C(1,1));-asind(C(1,3));atan2d(C(2,3),C(3,3))];
    elseif output_type=='Q'
        OUTPUT=dcm2quaternion(C);
    end
end

%% quaternion function
function [Q]=dcm2quaternion(INPUT)

% DCM_flag=0;
% if abs(det(INPUT)-1)>tol %if determinant is off by one more than tol, user is warned.(determinant must be near to one so the length of vector won't be changed)
%     DCM_flag=1;
% end
% if abs(det(INPUT)+1)<0.05 %if determinant is near -1, DCM is improper (its product with any vector must leave its length unchanged and does not reverse the vector)
%     error('Error: Input DCM(s) improper');
% end
% if DCM_flag==1
%     errordlg('Warning: Input DCM matrix determinant(s) off from 1 by more than tolerance.')
% end
DCM_flag=0;
Q=NaN(4,1);
denom=NaN(4,1);
% denom(1)=0.5*sqrt(1+INPUT(1,1)-INPUT(2,2)-INPUT(3,3));
% denom(2)=0.5*sqrt(1-INPUT(1,1)+INPUT(2,2)-INPUT(3,3));
% denom(3)=0.5*sqrt(1-INPUT(1,1)-INPUT(2,2)+INPUT(3,3));
denom(4)=0.5*sqrt(1+INPUT(1,1)+INPUT(2,2)+INPUT(3,3));
%determine which Q equations maximize denominator
% switch find(denom==max(denom),1,'first')  %determines max value of qtests to put in denominator
%     case 1
%         Q(1)=denom(1);
%         Q(2)=(INPUT(1,2)+INPUT(2,1))/(4*Q(1));
%         Q(3)=(INPUT(1,3)+INPUT(3,1))/(4*Q(1));
%         Q(4)=(INPUT(2,3)-INPUT(3,2))/(4*Q(1));
%     case 2
%         Q(2)=denom(2);
%         Q(1)=(INPUT(1,2)+INPUT(2,1))/(4*Q(2));
%         Q(3)=(INPUT(2,3)+INPUT(3,2))/(4*Q(2));
%         Q(4)=(INPUT(3,1)-INPUT(1,3))/(4*Q(2));
%     case 3
%         Q(3)=denom(3);
%         Q(1)=(INPUT(1,3)+INPUT(3,1))/(4*Q(3));
%         Q(2)=(INPUT(2,3)+INPUT(3,2))/(4*Q(3));
%         Q(4)=(INPUT(1,2)-INPUT(2,1))/(4*Q(3));
%     case 4
        Q(1)=denom(4); % Changed
        Q(2)=(INPUT(2,3)-INPUT(3,2))/(4*Q(1));
        Q(3)=(INPUT(3,1)-INPUT(1,3))/(4*Q(1));
        Q(4)=(INPUT(1,2)-INPUT(2,1))/(4*Q(1));
%end
clear denom
end
end
