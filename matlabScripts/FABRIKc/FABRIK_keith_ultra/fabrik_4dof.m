origin_position = [16.3651  -25.7480  124.7638]'/1000;
y_direction = -[0.0529   -0.9426    0.3296]';
axis_direction = [-0.9970   -0.0314    0.0703]';

% origin_position = [30.8518   30.1910  113.1720]'/1000;
% y_direction = [0.5272   -0.7739    0.3508]';
% axis_direction = [-0.8316   -0.5547    0.0260]';

end_position = [11.1823   -5.0772  122.6402]'/1000;
end_direction = [0.1214    0.9818   -0.1458]';

theta= 1.5664;%  0.7522; % 
Lg =  20.0012  ;%30; %
origin_direction = cos(theta)*axis_direction+y_direction*sin(theta);
origin_position = origin_position + axis_direction*(Lg-15)/1000;

% nor_direction = cross(y_direction,axis_direction);
% nor_direction = nor_direction/norm(nor_direction);
% tem_direction = end_position-(origin_position+axis_direction*15);
% nor_direction2 = cross(end_direction,tem_direction/norm(tem_direction));
% nor_direction2 = nor_direction2/norm(nor_direction2);
% origin_direction = cross(nor_direction,nor_direction2);
% % origin_direction = end_direction-tem_proj*nor_direction;
% origin_direction = origin_direction/norm(origin_direction);

% figure; 
hold on; grid on; axis equal;
xlabel('x');ylabel('y');zlabel('z');
view([75 25]);
color1 = rand(1,3); color2 = 1-color1;
% color1=[0 1 1];color2=[0 1 1];
p1 = [end_position end_direction/100+end_position];
plot3(p1(1,:),p1(2,:),p1(3,:),'Color',[0 1 0],LineWidth=1,LineStyle='--');
p2 = [origin_position origin_direction/100+origin_position];
plot3(p2(1,:),p2(2,:),p2(3,:),'Color',[1 0 0],LineWidth=1,LineStyle='--');
L = norm(end_position-origin_position)/2;
Li1=L/2;
Li2 = Li1;
p1o = origin_position;
p1j=p1o+Li1*origin_direction;
p1e=p1j+Li1*origin_direction;
[x,y,z]=pot2plot3(p1o,p1j,p1e,0);
pa = plot3(x,y,z,'Color',color1,LineWidth=2);
p2o=p1e;
p2j=p2o+Li2*origin_direction;
p2e=p2j+Li2*origin_direction;
[x,y,z]=pot2plot3(p2o,p2j,p2e,0);
pb = plot3(x,y,z,'Color',color2,LineWidth=2);
error_p=norm(p2e-end_position);
error_p0=0;
iter = 0;
while(error_p>1e-6)
    % deal with L
    if(abs(error_p0-error_p)<1e-3)
        l_tem = dot(-(p2e-end_position),end_direction);
        L=L+l_tem/2;
    end
    error_p0 = error_p;
    % backward phasing
    p2e = end_position;
    tem_pj = p2e-end_direction*Li2;
    tem_dir = (tem_pj-p1e)/norm(p1e-tem_pj);
    tem_po = tem_pj-tem_dir*Li2;
    [Li2,theta] = getL(tem_po,tem_pj,p2e,L);
    p2j=p2e-end_direction*Li2;
    p2o=p2j-tem_dir*Li2;
    [x,y,z]=pot2plot3(p2o,p2j,p2e,theta);
    set(pb,'XData',x','YData',y','ZData',z');
    pause(0.1);

    p1e=p2o;
    tem_pj = p2o-tem_dir*Li1;
    tem_dir2 = (tem_pj-p1o)/norm(p1o-tem_pj);
    tem_po = tem_pj-tem_dir2*Li1;
    [Li1,theta] = getL(tem_po,tem_pj,p1e,L);
    p1j=p1e-tem_dir*Li1;
    p1o=p1j-tem_dir2*Li1;
    [x,y,z]=pot2plot3(p1o,p1j,p1e,theta);
    set(pa,'XData',x','YData',y','ZData',z');
    pause(0.1);

    % forward phasing
    p1o = origin_position;
    tem_pj=p1o+Li1*origin_direction;
    tem_dir = -(tem_pj-p1e)/norm(p1e-tem_pj);
    tem_pe=tem_pj+Li1*tem_dir;
    [Li1,theta] = getL(p1o,tem_pj,tem_pe,L);
    p1j = p1o+Li1*origin_direction;
    p1e = p1j+Li1*tem_dir;
    [x,y,z]=pot2plot3(p1o,p1j,p1e,theta);
    set(pa,'XData',x','YData',y','ZData',z');
    pause(0.1);

    p2o=p1e;
    tem_pj=p2o+Li2*tem_dir;
    tem_dir2 = -(tem_pj-p2e)/norm(p2e-tem_pj);
    tem_pe=tem_pj+Li2*tem_dir2;
    [Li2,theta] = getL(p2o,tem_pj,tem_pe,L);
    p2j = p2o+Li2*tem_dir;
    p2e = p2j+Li2*tem_dir2;
    [x,y,z]=pot2plot3(p2o,p2j,p2e,theta);
    set(pb,'XData',x','YData',y','ZData',z');
    pause(0.1);

    error_p=norm(p2e-end_position);
    iter = iter + 1;
    disp(['The iteration is ', num2str(iter),'. And error is ',num2str(error_p)]);

end
%%
z1 = origin_direction/norm(origin_direction);
x1 = axis_direction;
y1 = cross(z1,x1)/norm(cross(z1,x1));
x1 = cross(y1,z1)/norm(cross(y1,z1));
T = [x1,y1,z1,origin_position;[0 0 0 1]];
L0 = L;
[theta1, delta1] = getThetaDelta(p1e,T,L0);
T2 = getT(theta1,delta1,L);
[theta2, delta2] = getThetaDelta(p2e,T*T2,L0);

SL = [L0,0,L0,0,0,0,0,0,0]';
psi = [0 L0 theta1 delta1 theta2 delta2]';
[Tend,S]=FKcc_2segs_bending_keith(psi,SL,1e-3);
PS_2segs_keith(S,SL,Tend,1e-3,T);
SL1000=SL*1000;
psi1000=psi;psi1000(2)=psi1000(2)*1000;
u = Psi2Curvature_keith(psi1000,SL1000);
function [theta, delta] = getThetaDelta(p,T,L)
    p0 = [p' 1]';
    p_r = eye(4)/T*p0;
    delta = atan(p_r(2)/p_r(1));
    p_z = p_r(3);
    f=@(x)(x*p_z-sin(x)*L)*10000;
    theta=fsolve(f,4);

end

function T = getT(theta1,delta1,L)
    k1 = theta1/L;
    if theta1==0
        T=[1 0 0 0
            0 1 0 0
            0 0 1 L
            0 0 0 1];
    else
        cosTHETA1=cos(theta1);sinTHETA1=sin(theta1);cosDELTA1=cos(delta1);sinDELTA1=sin(delta1);
        T=[(cosDELTA1)^2*(cosTHETA1-1)+1 sinDELTA1*cosDELTA1*(cosTHETA1-1) cosDELTA1*sinTHETA1 cosDELTA1*(1-cosTHETA1)/k1
            sinDELTA1*cosDELTA1*(cosTHETA1-1) (cosDELTA1)^2*(1-cosTHETA1)+cosTHETA1 sinDELTA1*sinTHETA1 sinDELTA1*(1-cosTHETA1)/k1
            -cosDELTA1*sinTHETA1 -sinDELTA1*sinTHETA1 cosTHETA1 sinTHETA1/k1
            0 0 0 1];

    end
end