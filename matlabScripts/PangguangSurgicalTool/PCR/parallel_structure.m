function PS0 = parallel_structure(is_refresh)
%   this is a function to set/get the parallel structure
%   input 1: for renew the parameter
% 
%   Author: Keith W.
%   Ver. 1.0
%   Date 05.15.2022
if(nargin == 0)
    is_refresh = 0;
end

persistent PS;
if(isempty(PS) || is_refresh)
    % initial disk arrangement dimension
    rod_number = 3;
    T_base = eye(4);
    T_base_1 = T_base * [eul2rotm([pi*0/3 0 0]) [0; 0; 0]; [0 0 0 1]];
    T_base_2 = T_base_1 \ T_base * [eul2rotm([pi*2/3 0 0]) [0; 0; 0]; [0 0 0 1]];
    T_base_3 = T_base_1 \ T_base * [eul2rotm([-pi*2/3 0 0]) [0; 0; 0]; [0 0 0 1]];
    ps = zeros(6, rod_number);
    aa1 = rotm2axang(T_base_1(1:3, 1:3));
    aa1 = aa1(1:3)'*aa1(4);
    aa2 = rotm2axang(T_base_2(1:3, 1:3)); 
    aa2 = aa2(1:3)'*aa2(4);
    aa3 = rotm2axang(T_base_3(1:3, 1:3));
    aa3 = aa3(1:3)'*aa3(4);
    ps(:,1) = [T_base_1(1:3,4);aa1];
    ps(:,2) = [T_base_2(1:3,4);aa2];
    ps(:,3) = [T_base_3(1:3,4);aa3];
    
    % end disk arrangement dimension
    T_end = eye(4);
    T_end_1 = T_end * [eul2rotm([pi*0/3 0 0]) [0; 0; 0]; [0 0 0 1]];
    T_end_2 = T_end_1 \ T_end * [eul2rotm([pi*2/3 0 0]) [0; 0; 0]; [0 0 0 1]];
    T_end_3 = T_end_1 \ T_end * [eul2rotm([-pi*2/3 0 0]) [0; 0; 0]; [0 0 0 1]];
    ds = zeros(6, rod_number);
    aa1 = rotm2axang(T_end_1(1:3, 1:3));
    aa1 = aa1(1:3)'*aa1(4);
    aa2 = rotm2axang(T_end_2(1:3, 1:3));
    aa2 = aa2(1:3)'*aa2(4);
    aa3 = rotm2axang(T_end_3(1:3, 1:3));
    aa3 = aa3(1:3)'*aa3(4);
    ds(:,1) = [T_end_1(1:3,4);aa1];
    ds(:,2) = [T_end_2(1:3,4);aa2];
    ds(:,3) = [T_end_3(1:3,4);aa3];

    % structure para
    sp.d = 0.4 * 1e-3;
    sp.E = 55e9;
    sp.mu = 0.33;
    sp.A = pi * sp.d ^ 2 / 4;
    sp.I = pi * sp.d ^ 4 / 64;
    sp.G = sp.E / 2 / (1 + sp.mu);sp.J = 2 * sp.I;
    sp.Kb = diag([sp.E * sp.I sp.E * sp.I 10]);
    sp.Ke = diag([3e8 3e8 sp.A * sp.E]);
    sp.L = 500e-3;

    PS.ps = ps;
    PS.ds = ds;
    PS.sp = sp;
end

PS0 = PS;

end

