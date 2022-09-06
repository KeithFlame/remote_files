function [proximal,distal] = continuum_points(xita,delta,q,number)
%	���������嶥�㣬���ڻ���

%���������
global L D;

%��������
vector_1 = [0.342020143325669;0;0.939692620785908];
vector_2 = [-0.171010071662834;0.296198132726024;0.939692620785908];
vector_3 = [-0.171010071662835;-0.296198132726024;0.939692620785908];

switch number
    case 1  %֧��1
        vector = vector_1;
    case 2  %֧��2
        vector = vector_2;
    case 3  %֧��3
        vector = vector_3;
end

if xita == 0    %��Ϊ0�����
    proximal = zeros(3,2);
    distal = zeros(3,2);
    
    proximal(:,1) = q * vector;
    proximal(:,2) = proximal(:,1) - [0;0;L];
    
    distal(:,1) = q * vector - [0;0;2*L+D];
    distal(:,2) = distal(:,1) + [0;0;L];
else            %�Ȳ�Ϊ0�����
    proximal = zeros(3,11);
    distal = zeros(3,11);
    
    base = q * vector;

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

