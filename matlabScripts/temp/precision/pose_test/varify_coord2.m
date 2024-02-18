% 坐标系验证
[Tt7, Tt19, Tt31] = registration_CW2('pM3','pM2',2);
% [Tt71, Tt191, Tt311] = registration_CW('Measure7',2);

err=zeros(6,2);

[C7, C19, C31]=getData71931('pM4',2);
T7=getCoord(C7);
T19=getCoord(C19);
T01 = T7/Tt7;
T02 = T19/Tt19;

err(1,1)=norm(T01(1:3,4)-T02(1:3,4));
axang=rotm2axang(T02(1:3,1:3)'*T01(1:3,1:3));
err(1,2)=axang(4)*180/pi;

[C7, C19, C31]=getData71931('pM5',2);
T31=getCoord(C31);
T19=getCoord(C19);
T01 = T31/Tt31;
T02 = T19/Tt19;
err(2,1)=norm(T01(1:3,4)-T02(1:3,4));
axang=rotm2axang(T02(1:3,1:3)'*T01(1:3,1:3));
err(2,2)=axang(4)*180/pi;

[C7, C19, C31]=getData71931('pM1',2);
T7=getCoord(C7);
T31=getCoord(C31);
T01 = T7/Tt7;
T02 = T31/Tt31;
err(3,1)=norm(T01(1:3,4)-T02(1:3,4));
axang=rotm2axang(T02(1:3,1:3)'*T01(1:3,1:3));
err(3,2)=axang(4)*180/pi;

[C7, C19, C31]=getData71931('pM6',2);
T7=getCoord(C7);
T31=getCoord(C31);
T01 = T7/Tt7;
T02 = T31/Tt31;
err(4,1)=norm(T01(1:3,4)-T02(1:3,4));
axang=rotm2axang(T02(1:3,1:3)'*T01(1:3,1:3));
err(4,2)=axang(4)*180/pi;

% [C7, C19, C31]=getData71931('M1');
% T7=getCoord(C7);
% T19=getCoord(C19);
% T31=getCoord(C31);
% T01 = T7/Tt7;
% T02 = T19/Tt19;
% T03 = T31/Tt31;
% err(4,1)=norm(T01(1:3,4)-T02(1:3,4));
% axang=rotm2axang(T02(1:3,1:3)'*T01(1:3,1:3));
% err(4,2)=axang(4)*180/pi;
% err(5,1)=norm(T01(1:3,4)-T03(1:3,4));
% axang=rotm2axang(T03(1:3,1:3)'*T01(1:3,1:3));
% err(5,2)=axang(4)*180/pi;
% err(6,1)=norm(T02(1:3,4)-T03(1:3,4));
% axang=rotm2axang(T03(1:3,1:3)'*T02(1:3,1:3));
% err(6,2)=axang(4)*180/pi;