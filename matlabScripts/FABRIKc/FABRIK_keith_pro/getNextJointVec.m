function n = getNextJointVec(l,v,aby)
    l = l/norm(l);
    v = v/norm(v);
    aby = aby/norm(aby);
    lv = cross(l,v);
    K1 = dot(aby,v);
    K2 = dot(l,v)*dot(aby,l);
    K3 = dot(aby,lv);
    
    Kn = norm([K1-K2,K3]);
    phi = atan(K3/(K1-K2));
    theta = acos(-K2/Kn)+phi;
    
    a1 = [cos(theta) dot(l,v)*(1-cos(theta)) sin(theta)];
    n1 = dot(a1,[v(1) l(1) lv(1)]);
    n2 = dot(a1,[v(2) l(2) lv(2)]);
    n3 = dot(a1,[v(3) l(3) lv(3)]); 
    n = [n1,n2,n3]';
end