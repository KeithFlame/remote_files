function T = fromQuat2T(vec)
T=eye(4);
T(1:3,4)=vec(1:3)/1000;
quat = vec(4:7)';
quat = quat/norm(quat);
Ro = quat2rotm(quat);
Ro_=Ro*eul2rotm([0 pi 0]);
T(1:3,1:3) = Ro_;
end