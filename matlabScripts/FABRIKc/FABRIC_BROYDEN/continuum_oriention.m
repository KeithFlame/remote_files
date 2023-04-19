function R_tb_te = continuum_oriention(xita,delta)
%   求解单段连续体末端姿态
R_tb_te=[cos(delta)^2*cos(xita)+sin(delta)^2  sin(delta)*cos(delta)*(1-cos(xita))  cos(delta)*sin(xita);
                     sin(delta)*cos(delta)*(1-cos(xita))  sin(delta)^2*cos(xita)+cos(delta)^2  -sin(delta)*sin(xita);
                     -cos(delta)*sin(xita)                 sin(delta)*sin(xita)                  cos(xita)];
end

