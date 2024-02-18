function [arm]=modify_trolley_Arm_Type2(arm, dL)
arm.d1=dL(1);
arm.d3=dL(2);
arm.a4=0;
arm.a5=-dL(3);
arm.d5=dL(4);
arm.a7=dL(3);
arm.d8=dL(5);
end