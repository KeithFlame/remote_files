function [g,h]=lessThanConstraint(x)
    L1=x(1);Lr=x(2);L2=x(3);Lg=x(4);
    K1=x(5)*0.1;K2=x(6)*0.1;E=x(7)*1e10/3;Lstem=x(8)*100;

    g=[-x(1)
       -x(2)
       -x(3)
       -x(4)
       -x(5)
       -x(6)
       3-K1
       3-x(6)
       (x(1)+x(2)+x(3)+x(4))-78
       70-(x(1)+x(2)+x(3)+x(4))

       ];
    h=[];
end
    