function [parasol] = C3IKv4(T,Lr,L2,Lg)
state=1;iter=0;

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
thres=3e-4; % 4.5 5e-2 5e-3 5e-4 1e-4 1e-5 1e-6
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
iter=k;
er=1;


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

parasol=[PHI;0;theta1;L1;DELTA1;Lr;theta2;L2;DELTA2;Lg];
end

function [parasol] = C4IKv5(T,theta1m,theta2m,costheta1m,Lsm,L1,Lr,L2,Lg)

nx=T(1,1);ny=T(2,1);nz=T(3,1);n=[nx;ny;nz];
    ax=T(1,3);ay=T(2,3);az=T(3,3);
    pxt=T(1,4);pyt=T(2,4);pzt=T(3,4);
% Independent parameters
thres=1e-2; % 2 1e-1 3e-2 3e-3 5e-4 5e-5 1e-6
k1=100;M=30;

% Case starts here
px=pxt-ax*Lg;py=pyt-ay*Lg;pz=pzt-az*Lg;
%% first iterate, and stop if theta break limit
a1=px*ax+py*ay;
a2=1-az^2;
a3=px^2+py^2;
hitlim=0;hitlim2=0;
theta1=acos(abs(az))+1e-3; % guarantee that D2>=0 ?!!!
costheta1=cos(theta1); % initial guess
cos2theta1=costheta1^2;
paththeta(:,1)=[theta1;0];
% out=false;
sw=-1; % begin from eqn2
if theta1<1e-4
    l1=L1*(1/2+theta1^2/24);
else
    l1=L1*tan(theta1/2)/theta1;
end
if theta1==acos(az)||theta1==acos(-az) % deal with degeneration
    B=2*((l1+Lr)*(cos2theta1-1)-a1);
    C=(l1+Lr)^2*(cos2theta1-1)+a3;
    l2=-C/B;
else
    A=cos2theta1-az^2;
    B=2*((l1+Lr)*(cos2theta1-1)-a1);
    C=(l1+Lr)^2*(cos2theta1-1)+a3;
    D2=B^2-4*A*C;
    D=real(sqrt(D2)); % find a way to use D to restrict range of theta1
    l2=(-B+sw*D)/(2*A);
end
l12r=l1+l2+Lr;a4=-a2*l2+a1;
g2=l2*acos(a4/l12r+az*costheta1)...
    *sqrt((l12r*(1+az*costheta1)+a4)/(l12r*(1-az*costheta1)-a4))...
    -L2;
paththeta(2,1)=g2;
k=0;
if abs(g2)<=thres % good guess
    Ls=pz-l2*az-l1-(l1+l2+Lr)*costheta1;
    costheta2=(a1+az*(pz-Ls)-l2-az*l1)/(l1+l2+Lr);
    theta2=acos(costheta2);
    sintheta1=sin(theta1);sintheta2=sin(theta2);
end
while abs(g2)>thres&&k<k1
    k=k+1;
    theta1d=theta1-1e-4; % finite differentiation
    if theta1d<0
        theta1d=theta1+1e-3;
    end
    costheta1d=cos(theta1d);cos2theta1d=costheta1d^2;
    if theta1d<1e-4
        l1d=L1*(1/2+theta1d^2/24);
    else
        l1d=L1*tan(theta1d/2)/theta1d;
    end
    Ad=cos2theta1d-az^2;
    Bd=2*((l1d+Lr)*(cos2theta1d-1)-a1);
    Cd=(l1d+Lr)^2*(cos2theta1d-1)+a3;
    Dd2=Bd^2-4*Ad*Cd;
    if Dd2<0
        theta1d=2*theta1-theta1d; % reverse differential direction
        if theta1d<1e-4
            l1d=L1*(1/2+theta1d^2/24);
        else
            l1d=L1*tan(theta1d/2)/theta1d;
        end
        costheta1d=cos(theta1d);cos2theta1d=costheta1d^2;
        Ad=cos2theta1d-az^2;
        Bd=2*((l1d+Lr)*(cos2theta1d-1)-a1);
        Cd=(l1d+Lr)^2*(cos2theta1d-1)+a3;
        Dd2=Bd^2-4*Ad*Cd;
    end
    Dd=real(sqrt(Dd2)); % find a way to use D to restrict range of theta1
    l2d=(-Bd+sw*Dd)/(2*Ad);
    l12rd=l1d+l2d+Lr;a4d=-a2*l2d+a1;
    g2d=l2d*acos(a4d/l12rd+az*costheta1d)...
        *sqrt((l12rd*(1+az*costheta1d)+a4d)/(l12rd*(1-az*costheta1d)-a4d))...
        -L2;
    dg2 = (g2-g2d)/(theta1-theta1d);
    m=-1;g2a=g2;
    step=g2/dg2;
    if step>0
        step=min([theta1 step]);
    else
        step=max([theta1-theta1m step]); % theta1 constrained and hitlim counted
    end
    while (g2a*g2<0||abs(g2a)>=abs(g2))&&m<M % forced down hill and no penetration
        m=m+1;
        theta1a=theta1-step;step=step/1.5; % scaling
        %             theta1a=min([theta1m,max([theta1a,0])]);
        costheta1a=cos(theta1a);cos2theta1a=costheta1a^2;
        if theta1a<=1e-4 %Taylor series expansion
            l1a=L1*(1/2+theta1a^2/24);
        else
            l1a=L1*tan(theta1a/2)/theta1a;
        end
        Aa=cos2theta1a-az^2;
        Ba=2*((l1a+Lr)*(cos2theta1a-1)-a1);
        Ca=(l1a+Lr)^2*(cos2theta1a-1)+a3;
        Da2=Ba^2-4*Aa*Ca;
        if Da2<0 % step too long
            continue
        end
        Da=sqrt(Da2); % find a way to use D to restrict range of theta1
        l2a=(-Ba+sw*Da)/(2*Aa);
        l12ra=l1a+l2a+Lr;a4a=-a2*l2a+a1;
        g2a=l2a*acos(a4a/l12ra+az*costheta1a)...
            *sqrt((l12ra*(1+az*costheta1a)+a4a)/(l12ra*(1-az*costheta1a)-a4a))...
            -L2;
    end
    g2=g2a;theta1=theta1a;costheta1=costheta1a;
    l1=l1a;l2=l2a;%m
    paththeta(:,k+1)=[theta1;g2];
    %         k
    %         abs(theta1-paththeta(1,k))
    if abs(g2-paththeta(2,k))<1e-5&&abs(g2)>1e-4 % switching or !! could be config limit !!
        hitlim=hitlim+1;
        if hitlim==3&&sw==-1
            hitlim=0;
            sw=1; % switch from eqn2 to eqn1
            if theta1<acos(abs(az)) % prevent eqn1 from crossing singularity
                theta1m=acos(abs(az))-1e-4; % don't forget to change it back later !!
            end
        elseif hitlim==3&&sw==1 % out of dex space
            theta1=0;theta2=0;Ls=0;validtheta=false;break
        end
    end
    
    if abs(g2)<thres % check at convergence for cases like #3989
        Ls=pz-l2*az-l1-(l1+l2+Lr)*costheta1;
        costheta2=(a1+az*(pz-Ls)-l2-az*l1)/(l1+l2+Lr);
        theta2=acos(costheta2);
        sintheta1=sin(theta1);sintheta2=sin(theta2);
        if theta2>theta2m||Ls<0||Ls>Lsm+0.01 % converged to the solution that violates config limit
            hitlim2=hitlim2+1;
            if hitlim2>1
                break
            end
            theta1=theta1m; % or acos(-abs(az)): move to the right side to pursue the other solution
            costheta1=costheta1m;cos2theta1=costheta1^2;
            if theta1<1e-4
                l1=L1*(1/2+theta1^2/24);
            else
                l1=L1*tan(theta1/2)/theta1;
            end
            if theta1==acos(az)||theta1==acos(-az) % deal with degeneration
                B=2*((l1+Lr)*(cos2theta1-1)-a1);
                C=(l1+Lr)^2*(cos2theta1-1)+a3;
                l2=-C/B;
            else
                A=cos2theta1-az^2;
                B=2*((l1+Lr)*(cos2theta1-1)-a1);
                C=(l1+Lr)^2*(cos2theta1-1)+a3;
                D2=B^2-4*A*C;
                D=real(sqrt(D2));
                l2=(-B+sw*D)/(2*A);
            end
            l12r=l1+l2+Lr;a4=-a2*l2+a1;
            g2=l2*acos(a4/l12r+az*costheta1)...
                *sqrt((l12r*(1+az*costheta1)+a4)/(l12r*(1-az*costheta1)-a4))...
                -L2;
            Ls=pz-l2*az-l1-(l1+l2+Lr)*costheta1;
            costheta2=(a1+az*(pz-Ls)-l2-az*l1)/(l1+l2+Lr);
            theta2=acos(costheta2);
            sintheta1=sin(theta1);sintheta2=sin(theta2);
        end
    end
    if k==k1 % out of dex space, and not stopped by minimum or config limit
        theta1=0;theta2=0;Ls=0;
    end
end

%% delta1 + phi
%same from here on
if theta1<=1e-4
    DELTA1plusPHI=0;
%     DELTA1plusPHI=para(1)+para(5);%use the last available delta
else
    DELTA1plusPHI=atan2(py-ay*l2,px-ax*l2);
end
cosDELTA1plusPHI=cos(DELTA1plusPHI);sinDELTA1plusPHI=sin(DELTA1plusPHI);

%% delta2 + phi
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

parasol=[PHI;Ls;theta1;L1;DELTA1;Lr;theta2;L2;DELTA2;Lg];
end