function [proximal,distal] = continuum_points_new(xita,delta,q,number)
%	计算连续体顶点，用于绘制

%连续体参数
global L D;
%点位置
global P_2_proximal_base P_3_proximal_base;
%导轨向量
vector_1 = [0.342020143325669;0;0.939692620785908];
vector_2 = [-0.171010071662834;0.296198132726024;0.939692620785908];
vector_3 = [-0.171010071662835;-0.296198132726024;0.939692620785908];

switch number
    case 1  %支链1
        vector = vector_1;
        base = q * vector;
    case 2  %支链2
        vector = vector_2;
        base = P_2_proximal_base;
    case 3  %支链3
        vector = vector_3;
        base = P_3_proximal_base;
end

if xita == 0    %θ为0的情况
    proximal = zeros(3,2);
    distal = zeros(3,2);
    
    proximal(:,1) = base;
    proximal(:,2) = proximal(:,1) - [0;0;L];
    
    distal(:,1) = base - [0;0;2*L+D];
    distal(:,2) = distal(:,1) + [0;0;L];
else            %θ不为0的情况
    proximal = zeros(3,11);
    distal = zeros(3,11);
    
   

    for i=0:10
        proximal(1,i+1) = L/xita * (cos(i/10 * xita) - 1) * cos(delta) + base(1);
        proximal(2,i+1) = -L/xita * (cos(i/10 * xita) - 1) * sin(delta) + base(2);
        proximal(3,i+1) = -L/xita * sin(i/10 * xita) + base(3);
    end
    
    end_ = base + [  2*L/xita*(cos(xita)-1)*cos(delta) - D*sin(xita)*cos(delta);
        -2*L/xita*(cos(xita)-1)*sin(delta) + D*sin(xita)*sin(delta);
        -2*L/xita*sin(xita) - D*cos(xita)];
    for i=0:10
        distal(1,i+1) = -L/xita * (cos(i/10 * xita) - 1) * cos(delta) + end_(1);
        distal(2,i+1) = L/xita * (cos(i/10 * xita) - 1) * sin(delta) + end_(2);
        distal(3,i+1) = L/xita * sin(i/10 * xita) + end_(3);
    end
end

end

