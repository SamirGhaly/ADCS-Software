function [u_b,v_b,w_b] =O2B(q,u_o,v_o,w_o)%orbital to body
N=sqrt(q(:,1).^2 + q(:,2).^2 + q(:,3).^2 + q(:,4).^2);
        qin=q./N;
        B2O= zeros(3,3);
        B2O(1,1,:) = qin(:,1).^2 + qin(:,2).^2 - qin(:,3).^2 - qin(:,4).^2;
        B2O(1,2,:) = 2.*(qin(:,2).*qin(:,3) + qin(:,1).*qin(:,4));
        B2O(1,3,:) = 2.*(qin(:,2).*qin(:,4) - qin(:,1).*qin(:,3));
        B2O(2,1,:) = 2.*(qin(:,2).*qin(:,3) - qin(:,1).*qin(:,4));
        B2O(2,2,:) = qin(:,1).^2 - qin(:,2).^2 + qin(:,3).^2 - qin(:,4).^2;
        B2O(2,3,:) = 2.*(qin(:,3).*qin(:,4) + qin(:,1).*qin(:,2));
        B2O(3,1,:) = 2.*(qin(:,2).*qin(:,4) + qin(:,1).*qin(:,3));
        B2O(3,2,:) = 2.*(qin(:,3).*qin(:,4) - qin(:,1).*qin(:,2));
        B2O(3,3,:) = qin(:,1).^2 - qin(:,2).^2 - qin(:,3).^2 + qin(:,4).^2;
        O2B=inv(B2O);
        V_b=O2B*[u_o,v_o,w_o];
        u_b=V_b(1);v_b=V_b(2);w_b=V_b(3);
end
