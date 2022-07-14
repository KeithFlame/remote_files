% to get maximum range of endeffector

I = imread("needleholder.png");
thresh = graythresh(I);
I2 = im2bw(I,thresh);

B=[0 1 0
    1 1 1
   0 1 0];
I2=imclose(I2,B);

imshowpair(I,A2,'montage');

% Ib=im2bw(I);
C = bwlabel(I2,4);
Ar = regionprops(C,'Area')
Ce = regionprops(C,'Centroid')