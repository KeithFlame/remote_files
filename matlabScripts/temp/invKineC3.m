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
 
