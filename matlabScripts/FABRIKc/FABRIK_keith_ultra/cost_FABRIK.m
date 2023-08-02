function f = cost_FABRIK(x)
theta1 = x(1);
Lg = x(2);

is_main=1;

if(is_main == 1)
    origin_position = [16.3651  -25.7480  124.7638]'/1000;
    y_direction = [0   1    0]';
    axis_direction = [-0.9970   -0.0314    0.0703]';
else
    origin_position = [30.8518   30.1910  113.1720]'/1000;
    y_direction = [0.5272   -0.7739    0]';
    axis_direction = [-0.8316   -0.5547    0.0260]';
end
tem_x = cross(y_direction,axis_direction);
y_direction = cross(axis_direction,tem_x);
y_direction = y_direction/norm(y_direction);

origin_position = origin_position + axis_direction*(Lg-15)/1000;
origin_direction = cos(theta1)*axis_direction+y_direction*sin(theta1);

end_position = [11.1823   -5.0772  122.6402]'/1000;
end_direction = is_main*[0.1214    0.9818   -0.1458]';

L = norm(end_position-origin_position)/2;
Li1=L/2;
Li2 = Li1;
p1o = origin_position;
p1j=p1o+Li1*origin_direction;
p1e=p1j+Li1*origin_direction;
p2o=p1e;
p2j=p2o+Li2*origin_direction;
p2e=p2j+Li2*origin_direction;
error_p=norm(p2e-end_position);
error_p0=0;
while(error_p>1e-5)
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
    [Li2] = getL(tem_po,tem_pj,p2e,L);
    p2j=p2e-end_direction*Li2;
    p2o=p2j-tem_dir*Li2;

    p1e=p2o;
    tem_pj = p2o-tem_dir*Li1;
    tem_dir2 = (tem_pj-p1o)/norm(p1o-tem_pj);
    tem_po = tem_pj-tem_dir2*Li1;
    [Li1] = getL(tem_po,tem_pj,p1e,L);
%     p1j=p1e-tem_dir*Li1;
%     p1o=p1j-tem_dir2*Li1;

    % forward phasing
    p1o = origin_position;
    tem_pj=p1o+Li1*origin_direction;
    tem_dir = -(tem_pj-p1e)/norm(p1e-tem_pj);
    tem_pe=tem_pj+Li1*tem_dir;
    [Li1] = getL(p1o,tem_pj,tem_pe,L);
    p1j = p1o+Li1*origin_direction;
    p1e = p1j+Li1*tem_dir;

    p2o=p1e;
    tem_pj=p2o+Li2*tem_dir;
    tem_dir2 = -(tem_pj-p2e)/norm(p2e-tem_pj);
    tem_pe=tem_pj+Li2*tem_dir2;
    [Li2] = getL(p2o,tem_pj,tem_pe,L);
    p2j = p2o+Li2*tem_dir;
    p2e = p2j+Li2*tem_dir2;

    error_p=norm(p2e-end_position);
    x',
end
f = L*2;
end