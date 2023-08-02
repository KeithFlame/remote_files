function p = invSkewMatrix_keith(R)
%this function gives the vector p from the skew-symmetric matrix R
if(length(R)==3)
p=zeros(3,1);
p(1)=R(3,2);
p(2)=R(1,3);
p(3)=R(2,1);
elseif(length(R)==4)
    p=zeros(6,1);
    R1=R(1:3,1:3);
    p(4)=R1(3,2);
	p(5)=R1(1,3);
    p(6)=R1(2,1);
    p(1:3,1)=R(1:3,4);
end

end