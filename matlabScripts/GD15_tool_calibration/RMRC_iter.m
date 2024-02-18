function [psi_dot]=RMRC_iter(J,x_dot,psi,seg_len)
%an iteration of Resolved Motion Rate Control
if(nargin==2)
    psi = [0 0 0 0 0 0]';
    seg_len = [100 10 20 15]';
end
if(norm(J(:,3)) ==0 && norm(J(:,4))==0)
    J = [J(:,1:2) J(:,5:6)];
    psi_size=4;
    Jv=J(1:3,:);
    Jw=J(4:6,:);
    v=x_dot(1:3,1);
    w=x_dot(4:6,1);
    invJv=svd_inv(Jv);
    psi_dot=invJv*v+(eye(psi_size)-invJv*Jv)*svd_inv(Jw*(eye(psi_size)-invJv*Jv))*(w-Jw*invJv*v);

    psi_dot = [psi_dot(1:2)' 0 0 psi_dot(3:4)']';
else
    invW = eye(6);
    invW(4:6,4:6)=invW(4:6,4:6)*10000;
    temp = J'*invW;
    invJ=svd_inv(temp*J)*temp;
    
    psi_dot = invJ*x_dot;

%     psi_size=6;
%     Jv=J(1:3,:);
%     Jw=J(4:6,:);
%     v=x_dot(1:3,1);
%     w=x_dot(4:6,1);
%     invJv=svd_inv(Jv);
%     psi_dot=invJv*v+(eye(psi_size)-invJv*Jv)*svd_inv(Jw*(eye(psi_size)-invJv*Jv))*(w-Jw*invJv*v);
end





end