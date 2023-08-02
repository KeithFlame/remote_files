function [Li,theta] = getL(po,pj,pe,L)
    l1 = (po-pj)/norm(pj-po);
    l2 = (pe-pj)/norm(pj-pe);
    tt=-dot(l1,l2);
    if(tt>1)
        tt=1;
    elseif(tt<-1)
        tt=-1;
    end

    theta = acos(tt);
    if(theta>1e-6)
        Li = L/theta*tan(theta/2);
    else
        Li=L/2;
    end
end