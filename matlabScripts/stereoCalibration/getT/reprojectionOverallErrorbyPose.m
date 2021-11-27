function error = reprojectionOverallErrorbyPose(poseX)

global A_new;
global TranslationC21; 
global Mpixeli;
global poseCam0;
gPixelsLeft=reshape(Mpixeli(1:18),[2 9])';
gPixelsRight=reshape(Mpixeli(19:36),[2 9])';
MPoints=[
      -5   5  1
       0   5  1
       5   5  1
      -5   0  1
       0   0  1
       5   0  1
      -5  -5  1
       0  -5  1
       5  -5  1
];
MPoints(:,3)=1;
rotRelative = rotRpy(poseX(1:3));
transRelative = [poseX(4); poseX(5); poseX(6)];

rotLeftNow = poseCam0(:,1:3) * rotRelative;
transLeftNow = poseCam0(:,4) + transRelative;

rotRightNow = rotLeftNow;
transRightNow = transLeftNow + TranslationC21;

HomographyLeftNow = A_new * [rotLeftNow(:,1), rotLeftNow(:,2), transLeftNow];
hLeftRow1 = HomographyLeftNow(1, :);
hLeftRow2 = HomographyLeftNow(2, :);
hLeftRow3 = HomographyLeftNow(3, :);


HomographyRightNow = A_new * [rotRightNow(:,1), rotRightNow(:,2), transRightNow];
hRightRow1 = HomographyRightNow(1, :);
hRightRow2 = HomographyRightNow(2, :);
hRightRow3 = HomographyRightNow(3, :);

error1 = 0;
for i = 1:size(gPixelsLeft, 1)
    error1 = error1 + norm(gPixelsLeft(i, :) - [hLeftRow1 * (MPoints(i, :)'), hLeftRow2 * (MPoints(i, :)')] / (hLeftRow3 * (MPoints(i, :)')));
    error1 = error1 + norm(gPixelsRight(i, :) - [hRightRow1 * (MPoints(i, :)'), hRightRow2 * (MPoints(i, :)')] / (hRightRow3 * (MPoints(i, :)')));
end
error = error1 / (2*size(gPixelsLeft, 1));

end