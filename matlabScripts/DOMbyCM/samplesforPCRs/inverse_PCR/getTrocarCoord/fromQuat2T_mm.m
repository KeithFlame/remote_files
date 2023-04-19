function T = fromQuat2T_mm(vec)
    T=eye(4);
    vec2=reshape(vec,[7 1]);
    T(1:3,4)=vec2(1:3);
    quat=vec(4:7);
    quat=quat/norm(quat);
    T(1:3,1:3)=quat2rotm(quat);
end