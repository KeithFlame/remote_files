function [Q,Psi,flag]=inverseCalib(x,Poses)
[psi1,flag(1)]=IK_solver(Poses(:,:,1),x);
[psi2,flag(2)]=IK_solver(Poses(:,:,2),x);
[psi3,flag(3)]=IK_solver(Poses(:,:,3),x);
[psi4,flag(4)]=IK_solver(Poses(:,:,4),x);
[psi5,flag(5)]=IK_solver(Poses(:,:,5),x);
Psi=[psi1 psi2 psi3 psi4 psi5];
if(sum(flag)==5)
    [q1,~]=calcQFromPsi(psi1,x);
    [q2,~]=calcQFromPsi(psi2,x);
    [q3,~]=calcQFromPsi(psi3,x);
    [q4,~]=calcQFromPsi(psi4,x);
    [q5,~]=calcQFromPsi(psi5,x);
    Q=[q1 q2 q3 q4 q5];
    flag=1;
else
    disp(["ERROR: IK not solved"]);
    Q=[];
    flag=0;
end
end