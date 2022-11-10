function psi = invKine_keith(T,arm1)
% This is a function to calculate the inverse kinematics for a 2-segment
% continuum robot with size is known.
% cite the approach by:
% Y. Wang, Z. Wu, L. Wang, B. Feng, and K. Xu, "Inverse Kinematics and Dexterous 
% Workspace Formulation for 2-Segment Continuum Robots with Inextensible Segments," 
% IEEE Robotics and Automation Letters, vol. 7, no. 1, pp. 510-517, 2022.
% 
% Author Keith W.
% Ver. 1.0
% Date: 25.03.2022

    LS=arm1.size_para;
    joint_limit = arm1.joint_limit;
    L1=LS(1);Lr=LS(2);L2=LS(3);Lg=LS(4);
    T(1:3,4)=T(1:3,4);
    config=invKineC3(T,Lr,L2,Lg);
    psi=[config(1) config(4) config(3) config(5) config(7) config(9)];
    if(config(4)>L1)
        config=invKineC4(T,L1,Lr,L2,Lg);
        psi=[config(1) config(2)+L1 config(3) config(5) config(7) config(9)];
    end
    psi(1)=psi(1)-LS(6);
    psi=limitRange(psi, joint_limit);
    t= psi(2);
    psi(2) = psi(1);
    psi(1) = t;
end

function config=invKineC3(T,Lr,L2,Lg)
%     state=1;iter=0;
    
    % Case starts here
    nx=T(1,1);ny=T(2,1);nz=T(3,1);n=[nx;ny;nz];
        ax=T(1,3);ay=T(2,3);az=T(3,3);
        pxt=T(1,4);pyt=T(2,4);pzt=T(3,4);
    % theta2<=theta2m, iteration will converge within limit
    px=pxt-ax*Lg;py=pyt-ay*Lg;pz=pzt-az*Lg;
    Pa=px*ax+py*ay+pz*az;
    A=-2*(Pa+Lr);
    B=px^2+py^2+pz^2-Lr^2;
    C=2*(1-az);
    D=2*(pz+Lr);
    f1=A+D+C*Lr;
    f2=D+A*az-Pa*C;
    f3=B+D*Lr;
    f4=az*B-Pa*D;
    
    % theta2
    theta2=pi/3; % initial guess
    costheta2=cos(theta2);sintheta2=sin(theta2);
    if theta2<=1e-4 %Taylor series expansion
        l2=L2*(1/2+theta2^2/24);
        dl2=L2*(theta2/6-theta2^3/120)/(costheta2+1);
    else
        l2=L2*tan(theta2/2)/theta2;
        dl2=L2*(theta2-sintheta2)/((costheta2+1)*theta2^2);
    end
    f = C*l2^2*(costheta2+1) + f1*costheta2*l2 + f2*l2 + f3*costheta2 + f4;
    thres=1e-4; % 4.5 5e-2 5e-3 5e-4 1e-4 1e-5 1e-6
    k=0;
    while abs(f)>thres&&k<100
        k=k+1;
        df = C*(2*l2*dl2*(costheta2+1)-l2^2*sintheta2) + f1*(costheta2*dl2-sintheta2*l2) + f2*dl2 - f3*sintheta2;
        theta2=theta2-f/df;
        
        theta2=min([2*pi/3,max([theta2,0])]); % constrained
        costheta2=cos(theta2);sintheta2=sin(theta2);
        if theta2<=1e-4 %Taylor series expansion
            l2=L2*(1/2+theta2^2/24);
            dl2=L2*(theta2/6-theta2^3/120)/(costheta2+1);
        else
            l2=L2*tan(theta2/2)/theta2;
            dl2=L2*(theta2-sintheta2)/((costheta2+1)*theta2^2);
        end
        f = C*l2^2*(costheta2+1) + f1*costheta2*l2 + f2*l2 + f3*costheta2 + f4;
    end
%     iter=k;
%     er=1;
    
    
    %% Binary judgement and adjustment
    l1=(A*l2+B)/(C*l2+D);
    
    costheta1=(pz-az*l2-l1)/(l1+l2+Lr);
    if costheta1>1
        costheta1=1;
    end
    theta1=acos(costheta1);
    
    if theta1<=1e-4
        L1=l1/(1/2+theta1^2/24);
    else
        L1=l1*theta1/tan(theta1/2);
    end
    
    %% delta1 + phi
    %same from here on
    if theta1<=1e-4
        DELTA1plusPHI=0;
    %     DELTA1plusPHI=para(1)+para(5);%%use the last available delta
    else
        DELTA1plusPHI=atan2(py-ay*l2,px-ax*l2);
    end
    
    % cache
    cosDELTA1plusPHI=cos(DELTA1plusPHI);sinDELTA1plusPHI=sin(DELTA1plusPHI);
    sintheta1=sin(theta1); %costheta1=cos(theta1);
    
    %% delta2 + phi
    % T_1e_2e = T_1b_1e\T
    if theta2<=1e-4
        DELTA2plusPHI=0;
    %     DELTA2plusPHI=para(1)+para(9);%%use the last available delta
    else
        if theta1<1e-4
            p3x=(cosDELTA1plusPHI^2*costheta1+sinDELTA1plusPHI^2)*px+sinDELTA1plusPHI*cosDELTA1plusPHI*(costheta1-1)*py-cosDELTA1plusPHI*sintheta1*pz+cosDELTA1plusPHI*L1*(theta1/2-theta1^3/24);
            p3y=sinDELTA1plusPHI*cosDELTA1plusPHI*(costheta1-1)*px+(sinDELTA1plusPHI^2*costheta1+cosDELTA1plusPHI^2)*py-sinDELTA1plusPHI*sintheta1*pz+sinDELTA1plusPHI*L1*(theta1/2-theta1^3/24);
        else
            p3x=(cosDELTA1plusPHI^2*costheta1+sinDELTA1plusPHI^2)*px+sinDELTA1plusPHI*cosDELTA1plusPHI*(costheta1-1)*py-cosDELTA1plusPHI*sintheta1*pz+cosDELTA1plusPHI*(1-costheta1)*L1/theta1;
            p3y=sinDELTA1plusPHI*cosDELTA1plusPHI*(costheta1-1)*px+(sinDELTA1plusPHI^2*costheta1+cosDELTA1plusPHI^2)*py-sinDELTA1plusPHI*sintheta1*pz+sinDELTA1plusPHI*(1-costheta1)*L1/theta1;
        end
        DELTA2plusPHI=atan2(p3y,p3x);
    end
    
    %cache
    cosDELTA2plusPHI=cos(DELTA2plusPHI);sinDELTA2plusPHI=sin(DELTA2plusPHI);
    
    %% phi
    R2=[cosDELTA1plusPHI^2*(costheta1-1)+1 sinDELTA1plusPHI*cosDELTA1plusPHI*(costheta1-1) cosDELTA1plusPHI*sintheta1
        sinDELTA1plusPHI*cosDELTA1plusPHI*(costheta1-1) cosDELTA1plusPHI^2*(1-costheta1)+costheta1 sinDELTA1plusPHI*sintheta1
        -cosDELTA1plusPHI*sintheta1 -sinDELTA1plusPHI*sintheta1 costheta1];
    R4=[cosDELTA2plusPHI^2*(costheta2-1)+1 sinDELTA2plusPHI*cosDELTA2plusPHI*(costheta2-1) cosDELTA2plusPHI*sintheta2
        sinDELTA2plusPHI*cosDELTA2plusPHI*(costheta2-1) cosDELTA2plusPHI^2*(1-costheta2)+costheta2 sinDELTA2plusPHI*sintheta2
        -cosDELTA2plusPHI*sintheta2 -sinDELTA2plusPHI*sintheta2 costheta2];
    R04=R2*R4;
    n4=R04'*n;
    PHI=atan2(n4(2),n4(1));
    
    DELTA1=DELTA1plusPHI-PHI;DELTA2=DELTA2plusPHI-PHI;
    if DELTA1>pi
        DELTA1=DELTA1-2*pi;
    elseif DELTA1<-pi
        DELTA1=DELTA1+2*pi;
    end
    if DELTA2>pi
        DELTA2=DELTA2-2*pi;
    elseif DELTA2<-pi
        DELTA2=DELTA2+2*pi;
    end
    
    config=[PHI;0;theta1;L1;DELTA1;Lr;theta2;L2;DELTA2;Lg];
    end
    
function config=invKineC4(T,L1,Lr,L2,Lg)
% hyper parameters
    thres=1e-4;k1=50;k2=100;M=10;Rs=5;
    
    nx=T(1,1);ny=T(2,1);nz=T(3,1);n=[nx;ny;nz];
    ax=T(1,3);ay=T(2,3);az=T(3,3);
%     a=[ax;ay;az];
    pxe=T(1,4);pye=T(2,4);pze=T(3,4);
    px=pxe-ax*Lg;py=pye-ay*Lg;pz=pze-az*Lg;
    
    % theta
    a1=px*ax+py*ay;
    a2=1-az^2;
    a3=px^2+py^2;
    
    run=0;
%     notconvergent=false;
    iter_run=zeros(1,10);
    theta1=pi/4;theta2=pi/3;costheta1=cos(theta1);costheta2=cos(theta2); % initial guess
    while run<Rs
        k=0;
        if theta1<1e-4
            l1=L1*(1/2+theta1^2/24);
            dl1=-L1/12;
        else
            l1=L1*tan(theta1/2)/theta1;
            dl1=L1*(sin(theta1)-theta1)/(tan(theta1/2)*(costheta1+1)^2*theta1^2);
        end
        if theta2<1e-4
            l2=L2*(1/2+theta2^2/24);
            dl2=-L2/12;
        else
            l2=L2*tan(theta2/2)/theta2;
            dl2=L2*(sin(theta2)-theta2)/(tan(theta2/2)*(costheta2+1)^2*theta2^2);
        end
        %cache
        c1=l1+l2+Lr;
        c2=c1^2-(a3+a2*l2^2-2*a1*l2);
        c3=a1-a2*l2;
        c4=c1*costheta1;
        c6=costheta2 - az*costheta1;
        
        g1 = c4^2 - c2;
        g2 = a2*l2 + c1*c6 - a1;
        while (abs(g1)>thres||abs(g2)>thres)&&k<k1
            k=k+1;
            
            dg1d1 = 2*(c4*(dl1*costheta1+c1)-c1*dl1);
            dg1d2 = 2*(c4*dl2*costheta1-(c1+c3)*dl2);
            dg2d1 = -az*c1 + c6*dl1;
            dg2d2 = a2*dl2 + c1 + c6*dl2;
            
            detJ=dg1d1*dg2d2-dg1d2*dg2d1;
            invJ=[dg2d2 -dg1d2;-dg2d1 dg1d1]/detJ;
            R=-invJ*[g1;g2];
            m=-1;g1a=g1;g2a=g2;%down hill
            while (abs(g1a)>=abs(g1)||abs(g2a)>=abs(g2))&&m<M
                m=m+1;
                costheta1a=costheta1+R(1)/(2^m);costheta2a=costheta2+R(2)/(2^m);
                costheta1a=min([1,costheta1a]);costheta1a=max([0,costheta1a]);
                costheta2a=min([1,costheta2a]);costheta2a=max([-0.5,costheta2a]);
                theta1=acos(costheta1a);theta2=acos(costheta2a);
                
                sintheta1a=sqrt(1-costheta1a^2);%sin2theta1=sin(2*theta1a);%cos2theta1=cos(2*theta1);
                sintheta2a=sqrt(1-costheta2a^2);%sin2theta2=sin(2*theta2a);%cos2theta2=cos(2*theta2);
                if theta1<1e-4
                    l1=L1*(1/2+theta1^2/24);
                    dl1=-L1/12;
                else
                    l1=L1*tan(theta1/2)/theta1;
                    dl1=L1*(sintheta1a-theta1)/(tan(theta1/2)*(costheta1a+1)^2*theta1^2);
                end
                if theta2<1e-4
                    l2=L2*(1/2+theta2^2/24);
                    dl2=-L2/12;
                else
                    l2=L2*tan(theta2/2)/theta2;
                    dl2=L2*(sintheta2a-theta2)/(tan(theta2/2)*(costheta2a+1)^2*theta2^2);
                end
                %cache
                c1=l1+l2+Lr;
                c2=c1^2-(a3+a2*l2^2-2*a1*l2);
                c3=a1-a2*l2;
                c4=c1*costheta1a;
                c6=costheta2a - az*costheta1a;
                
                g1a = c4^2 - c2;
                g2a = a2*l2 + c1*c6 - a1;
            end
            costheta1=costheta1a;costheta2=costheta2a;g1=g1a;g2=g2a;
        end
        c5=c1*costheta2 - c3;
        g2 = c5^2 - c2*az^2;
        
        if abs(g1)>thres||abs(g2)>thres % change functions, f1, f2
            while (abs(g1)>thres||abs(g2)>thres)&&k<k2
                k=k+1;
                
                db2d1 = 2*c1*dl1;
                db2d2 = 2*(c1+c3)*dl2;
                dg1d1 = 2*c4*(dl1*costheta1+c1)-db2d1;
                dg1d2 = 2*c4*dl2*costheta1-db2d2;
                dg2d1 = 2*c5*dl1*costheta2-(az^2)*db2d1;
                dg2d2 = 2*c5*(dl2*costheta2+c1+a2*dl2)-(az^2)*db2d2;
                
                detJ=dg1d1*dg2d2-dg1d2*dg2d1;
                invJ=[dg2d2 -dg1d2;-dg2d1 dg1d1]/detJ;
                R=-invJ*[g1;g2];
                m=-1;g1a=g1;g2a=g2;%down hill
                while (abs(g1a)>=abs(g1)||abs(g2a)>=abs(g2))&&m<M
                    m=m+1;
                    costheta1a=costheta1+R(1)/(2^m);costheta2a=costheta2+R(2)/(2^m);
                    costheta1a=min([1,costheta1a]);costheta1a=max([0,costheta1a]);
                    costheta2a=min([1,costheta2a]);costheta2a=max([-0.5,costheta2a]);
                    theta1=acos(costheta1a);theta2=acos(costheta2a);
                    
                    sintheta1a=sqrt(1-costheta1a^2);%sin2theta1=sin(2*theta1a);%cos2theta1=cos(2*theta1);
                    sintheta2a=sqrt(1-costheta2a^2);%sin2theta2=sin(2*theta2a);%cos2theta2=cos(2*theta2);
                    if theta1<1e-4
                        l1=L1*(1/2+theta1^2/24);
                        dl1=-L1/12;
                    else
                        l1=L1*tan(theta1/2)/theta1;
                        dl1=L1*(sintheta1a-theta1)/(tan(theta1/2)*(costheta1a+1)^2*theta1^2);
                    end
                    if theta2<1e-4
                        l2=L2*(1/2+theta2^2/24);
                        dl2=-L2/12;
                    else
                        l2=L2*tan(theta2/2)/theta2;
                        dl2=L2*(sintheta2a-theta2)/(tan(theta2/2)*(costheta2a+1)^2*theta2^2);
                    end
                    %cache
                    c1=l1+l2+Lr;
                    c2=c1^2-(a3+a2*l2^2-2*a1*l2);
                    c3=a1-a2*l2;
                    c4=c1*costheta1a;
                    c5=c1*costheta2a - c3;
                    
                    g1a = c4^2 - c2;
                    g2a = c5^2 - c2*az^2;
                end
                costheta1=costheta1a;costheta2=costheta2a;g1=g1a;g2=g2a;
            end
        end
        c6=costheta2 - az*costheta1;
        h = a2*l2 + c1*c6 - a1;
        run=run+1;
        iter_run(run)=k;
        
        if abs(g1)>thres||abs(g2)>thres||abs(h)>thres %ep(ind)>0.1
            if run==Rs
%                 notconvergent=true;
                break;
            end
            vec=2*pi*rand(1);
            costheta1=costheta1+0.5*sin(vec);
            costheta2=costheta2+0.5*cos(vec);
            costheta1=min([1,costheta1]);costheta1=max([-1,costheta1]);
            costheta2=min([1,costheta2]);costheta2=max([-1,costheta2]);
        else
            break
        end
    end
    theta1=acos(costheta1);theta2=acos(costheta2);
    sintheta1=sintheta1a;sintheta2=sintheta2a;
    Ls=-l2*az+pz-l1-(l1+l2+Lr)*costheta1;
    
    % delta1 + phi
    if theta1<=1e-4
        DELTA1plusPHI=0;
    %     DELTA1plusPHI=para(1)+para(5);%use the last available delta
    else
        DELTA1plusPHI=atan2(py-ay*l2,px-ax*l2);
    end
    cosDELTA1plusPHI=cos(DELTA1plusPHI);sinDELTA1plusPHI=sin(DELTA1plusPHI);
    
    % delta2 + phi
    % T_1e_2e = T_w_1e\T
    if theta2<=1e-4
        DELTA2plusPHI=0;
    %     DELTA2plusPHI=para(1)+para(9);%use the last available delta
    else
        if theta1<=1e-4
            p3x=((cosDELTA1plusPHI)^2*costheta1+(sinDELTA1plusPHI)^2)*px+sinDELTA1plusPHI*cosDELTA1plusPHI*(costheta1-1)*py-cosDELTA1plusPHI*sintheta1*pz+cosDELTA1plusPHI*L1*(theta1/2-theta1^3/24)+Ls*cosDELTA1plusPHI*sintheta1;
            p3y=sinDELTA1plusPHI*cosDELTA1plusPHI*(costheta1-1)*px+((sinDELTA1plusPHI)^2*costheta1+(cosDELTA1plusPHI)^2)*py-sinDELTA1plusPHI*sintheta1*pz+sinDELTA1plusPHI*L1*(theta1/2-theta1^3/24)+Ls*sinDELTA1plusPHI*sintheta1;
        else
            p3x=((cosDELTA1plusPHI)^2*costheta1+(sinDELTA1plusPHI)^2)*px+sinDELTA1plusPHI*cosDELTA1plusPHI*(costheta1-1)*py-cosDELTA1plusPHI*sintheta1*pz+cosDELTA1plusPHI*(1-costheta1)*L1/theta1+Ls*cosDELTA1plusPHI*sintheta1;
            p3y=sinDELTA1plusPHI*cosDELTA1plusPHI*(costheta1-1)*px+((sinDELTA1plusPHI)^2*costheta1+(cosDELTA1plusPHI)^2)*py-sinDELTA1plusPHI*sintheta1*pz+sinDELTA1plusPHI*(1-costheta1)*L1/theta1+Ls*sinDELTA1plusPHI*sintheta1;
        end
        DELTA2plusPHI=atan2(p3y,p3x);
    end
    cosDELTA2plusPHI=cos(DELTA2plusPHI);sinDELTA2plusPHI=sin(DELTA2plusPHI);
    
    R2=[(cosDELTA1plusPHI)^2*(costheta1-1)+1 sinDELTA1plusPHI*cosDELTA1plusPHI*(costheta1-1) cosDELTA1plusPHI*sintheta1
        sinDELTA1plusPHI*cosDELTA1plusPHI*(costheta1-1) (cosDELTA1plusPHI)^2*(1-costheta1)+costheta1 sinDELTA1plusPHI*sintheta1
        -cosDELTA1plusPHI*sintheta1 -sinDELTA1plusPHI*sintheta1 costheta1];
    R4=[(cosDELTA2plusPHI)^2*(costheta2-1)+1 sinDELTA2plusPHI*cosDELTA2plusPHI*(costheta2-1) cosDELTA2plusPHI*sintheta2
        sinDELTA2plusPHI*cosDELTA2plusPHI*(costheta2-1) (cosDELTA2plusPHI)^2*(1-costheta2)+costheta2 sinDELTA2plusPHI*sintheta2
        -cosDELTA2plusPHI*sintheta2 -sinDELTA2plusPHI*sintheta2 costheta2];
    R04=R2*R4;
    n4=R04'*n;
    PHI=atan2(n4(2),n4(1));
    
    DELTA1=DELTA1plusPHI-PHI;DELTA2=DELTA2plusPHI-PHI;
    if DELTA1>pi
        DELTA1=DELTA1-2*pi;
    elseif DELTA1<-pi
        DELTA1=DELTA1+2*pi;
    end
    if DELTA2>pi
        DELTA2=DELTA2-2*pi;
    elseif DELTA2<-pi
        DELTA2=DELTA2+2*pi;
    end
    
    config=[PHI;Ls;theta1;L1;DELTA1;Lr;theta2;L2;DELTA2;Lg];
end

function psi_l=limitRange(psi, joint_limit)
    psi_l=psi;
    while(psi_l(1)<joint_limit(1)||psi_l(1)>joint_limit(2))
        if(psi(1)<joint_limit(1))
            psi_l(1)=psi_l(1)+2*pi;
        elseif(psi(1)>joint_limit(2))
            psi_l(1)=psi_l(1)-2*pi;
        end
    end
    if(psi(2)<joint_limit(3))
        psi_l(2)=joint_limit(3);
    elseif(psi(2)>joint_limit(4))
        psi_l(2)=joint_limit(4);
    end
    if(psi(3)<joint_limit(5))
        psi_l(3)=joint_limit(5);
    elseif(psi(3)>joint_limit(6))
        psi_l(3)=joint_limit(6);
    end
    if(psi(3)/psi(2)>joint_limit(6)/joint_limit(4))
        psi_l(3)=joint_limit(6)/joint_limit(4)*psi(2);
    end
    while(psi_l(4)<joint_limit(7)||psi_l(4)>joint_limit(8))
        if(psi(4)<joint_limit(7))
            psi_l(4)=psi_l(4)+2*pi;
        elseif(psi(4)>joint_limit(8))
            psi_l(4)=psi_l(4)-2*pi;
        end
    end
    if(psi(5)<joint_limit(9))
        psi_l(5)=joint_limit(9);
    elseif(psi(5)>joint_limit(10))
        psi_l(5)=joint_limit(10);
    end
    while(psi_l(6)<joint_limit(11)||psi_l(6)>joint_limit(12))
        if(psi(6)<joint_limit(11))
            psi_l(6)=psi_l(6)+2*pi;
        elseif(psi(6)>joint_limit(12))
            psi_l(6)=psi_l(6)-2*pi;
        end
    end
end
