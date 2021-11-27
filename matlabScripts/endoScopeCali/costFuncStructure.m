function f= costFuncStructure(x)
x(5)=x(5)/10;x(6)=x(6)/10;
x(7)=x(7)*1e10/3;
x(8)=x(8)*100;

[endoQ,endoQ_,~]=getBetterObject(x);
cter1=0;
for i = 1:size(endoQ,1)
    cter1=sum(1e3*norm(endoQ_(i,:)-endoQ(i,:)))+cter1;
end
f=cter1;
end