% this is a function to eliminate the no-relative pixels

I=imread('F:\synchronism_plus\synchronism_plus\好好写文章\Thesis_Proposal\my own\pictures\欧拉伯努利梁2.tif');
[m,n,d]=size(I);
I1=I;
for i =1:m
    for j =1:n
%         if(I(i,j,1)==I(i,j,2)&&I(i,j,2)==I(i,j,3)&&I(i,j,2)<200)
        if((I(i,j,2)-I(i,j,1))>30)
            I1(i,j,:)=[255 255 255]';
        end
    end
end
imshowpair(I,I1,'montage');