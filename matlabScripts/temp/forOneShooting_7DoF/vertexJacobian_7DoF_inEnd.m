function [Jac, Ni] = vertexJacobian_7DoF_inEnd(T_base,arm,Q)
%INVKINE_7DOF inverse kinematics for a 7—DoF robot
% input1: robot origin (T_base)
% input2: robot parameter (arm)
% input3: robot acutuation (Q)
%
% output1: vertex jacobian (Jac)
% output2: Ni (Jac)

d_size = 1e-6;

[~,~,~,~,~,~,~,~,Tbase1,Tbase2,Tbase3,Tbase4,Tbase5,Tbase6,Tbase7,~]=FK_7DOF_Offset(arm,T_base,Q);
Ttar = Tbase7;
Tcur6 = Tbase6\Ttar;Tcur5 = Tbase5\Ttar;Tcur4 = Tbase4\Ttar;
Tcur3 = Tbase3\Ttar;Tcur2 = Tbase2\Ttar;Tcur1 = Tbase1\Ttar;
Tcur0 = T_base\Ttar;
Pcur = [Tcur6(1:3,4);Tcur5(1:3,4);Tcur4(1:3,4);Tcur3(1:3,4);...
    Tcur2(1:3,4);Tcur1(1:3,4); Tcur0(1:3,4)];
dGuess = eye(7)*d_size;
% Jac_ = zeros(21,7);
% for i = 7:-1:1
%     [~,~,~,~,~,~,~,~,Tbase1_,Tbase2_,...
%         Tbase3_,Tbase4_,Tbase5_,Tbase6_,Tbase7_,~]=FK_7DOF_Offset(arm,T_base,Q+dGuess(:,i));
%     Ttar = Tbase7_;
%     Tcur6_ = Tbase6_\Ttar;Tcur5_ = Tbase5_\Ttar;Tcur4_ = Tbase4_\Ttar;
%     Tcur3_ = Tbase3_\Ttar;Tcur2_ = Tbase2_\Ttar;Tcur1_ = Tbase1_\Ttar;
%     Tcur0_ = T_base\Ttar;
%     Pcur_ = [Tcur6_(1:3,4);Tcur5_(1:3,4);Tcur4_(1:3,4);Tcur3_(1:3,4);...
%     Tcur2_(1:3,4);Tcur1_(1:3,4);Tcur0_(1:3,4)];
%     Jac_(:,8-i) = (Pcur-Pcur_)/d_size;
% end

Jac = zeros(21,7);
for i = 1:7
    [~,~,~,~,~,~,~,~,Tbase1_,Tbase2_,...
        Tbase3_,Tbase4_,Tbase5_,Tbase6_,Tbase7_,~]=FK_7DOF_Offset(arm,T_base,Q+dGuess(:,i));
    Ttar = Tbase7_;
    Tcur6_ = Tbase6_\Ttar;Tcur5_ = Tbase5_\Ttar;Tcur4_ = Tbase4_\Ttar;
    Tcur3_ = Tbase3_\Ttar;Tcur2_ = Tbase2_\Ttar;Tcur1_ = Tbase1_\Ttar;
    Tcur0_ = T_base\Ttar;
    Pcur_ = [Tcur6_(1:3,4);Tcur5_(1:3,4);Tcur4_(1:3,4);Tcur3_(1:3,4);...
    Tcur2_(1:3,4);Tcur1_(1:3,4);Tcur0_(1:3,4)];
    Jac(:,i) = (Pcur-Pcur_)/d_size;
end
Ni=zeros(21,21);
P6=Tcur6(1:3,4)/norm(Tcur6(1:3,4));
P5 = (Tcur5(1:3,4)-Tcur6(1:3,4))/norm((Tcur5(1:3,4)-Tcur6(1:3,4)));
P4 = (Tcur4(1:3,4)-Tcur5(1:3,4))/norm((Tcur4(1:3,4)-Tcur5(1:3,4)));
P3 = (Tcur3(1:3,4)-Tcur4(1:3,4))/norm((Tcur3(1:3,4)-Tcur4(1:3,4)));
P2 = (Tcur2(1:3,4)-Tcur3(1:3,4))/norm((Tcur2(1:3,4)-Tcur3(1:3,4)));
P1 = (Tcur1(1:3,4)-Tcur2(1:3,4))/norm((Tcur1(1:3,4)-Tcur2(1:3,4)));
P0 = (Tcur0(1:3,4)-Tcur1(1:3,4))/norm((Tcur0(1:3,4)-Tcur1(1:3,4)));
Ni(1:3,1:3) = eye(3)-P6*P6';
Ni(4:6,4:6) = eye(3)-P5*P5';
Ni(7:9,7:9) = eye(3)-P4*P4';
Ni(10:12,10:12) = eye(3)-P3*P3';
Ni(13:15,13:15) = eye(3)-P2*P2';
Ni(16:18,16:18) = eye(3)-P1*P1';
Ni(19:21,19:21) = eye(3)-P0*P0';

end