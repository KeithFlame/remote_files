I0=imread('F:\synchronism_plus\synchronism_plus\好好写文章\好好写文章01\pic\final\f8(b)v1.tiff');
I1=I0;
[m,n,p]=size(I0);
for i =1:m
    for j = 1:n
            if(100<I0(i,j,1)&&I0(i,j,1)<200&&100<I0(i,j,2)&&I0(i,j,2)<200&&100<I0(i,j,3)&&I0(i,j,3)<200)
                I1(i,j,1)=255;
                I1(i,j,2)=255;
                I1(i,j,3)=255;
            end
    end
end
imshowpair(I0,I1,'montage');
imwrite(I1,"f81.tiff");