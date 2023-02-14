SL = [100 10 20 15 0 0 0 ]';
T = eye(4);T(1:3,4) = [60 0 100]';
[psi, flag] = IK_2segs_nobending_keith(T, SL);
[T,S] = FK_2segs_nobending_keith(psi, SL);
PS_2segs_nobending_keith(S,T);

