function [lt,ddt,theta_t]=getArcLength(psi,seg_i)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.11.2021
% Ver. 1.0
% purpose: calc this continuum length in trocar.
% input1: psi=[phi L theta1 delta1 theta2 delta2]
% input2: seg_i, this continuum's serials 
% output1: lt, the arc length in trocar
% output2: ddt, the origin of starting curving 
% output3: theta_t, the angle of arc in trocar 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

    if(seg_i==1)
        [r_L1, L1, ~, Lr, ~, L2, r_L2, Lg, r_trocar,r_Lstem]=getToolArmStructureParameter;
        [~, lf, theta, ~, ~, ~]=getPsi(psi);
        l_out=lf-Lr-L2-Lg;
        l_in=L1-l_out;
        r_L=max([r_L1, r_Lstem]);
        r_L_curve=r_L1;
    elseif(seg_i==2)
        [r_L1, ~, r_Lr, ~, r_L2, L2, ~, Lg,r_trocar, r_Lstem]=getToolArmStructureParameter;
        [~, lf, ~, ~, theta, ~]=getPsi(psi);
        l_out=lf-Lg;
        l_in=L2-l_out;
        r_L=max([r_L1, r_Lr, r_L2, r_Lstem]);
        r_L_curve=r_L2;
    end

    %% if theta == 0
    if(theta==0)
        lt=l_in;
        ddt=0;
        theta_t=0;
        return;
    end
    %% if C2、C4
    if(l_in<0)
        lt=0;
        ddt=0;
        theta_t=0;
        return;
    end
    %% if C1、C3
    r_curvature_target=l_in/theta;
    dt=2*r_trocar - r_L-r_L_curve;
    d=l_in;
    r_curvature_actual_limit=r_L_curve+(d*d+dt*dt)/2/d;
    if(r_curvature_actual_limit>r_curvature_target)
        % 靠边桀了
        theta_t=atan(dt/d)*2;
        lt=r_curvature_actual_limit*theta_t;
        ddt=dt;
    else
        % 不是边界
        thetai=atan(dt/d);
        if(theta>thetai)
            theta_=thetai;
        else
            theta_=theta;
        end
        ddt=r_trocar - r_L_curve+(r_trocar - r_L)*0.5*(1-cos(theta_/thetai*pi));
        theta_t=asin(d/r_curvature_target);
        lt=r_curvature_target*theta_t;
    end
end