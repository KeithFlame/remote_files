function [T,endoPsi]=getTestDataFrom202001
endoPsi=[
0 0 0 0 75 0;
0 0 0 0 -75 0;
0 0 0 0 30 0;
0 0 45 0 -60 0;
0 0 45 0 15 0;
0 0 45 0 60 0;
0 0 -45 0 75 0;
0 0 -45 0 0 0;
0 0 -45 0 -45 0 ;
]*pi/180;
T=zeros(4,4,9);
T(:,:,1)=[    0.2546    0.0164    0.9669   37.2724
   -0.0063    0.9999   -0.0153    0.4117
   -0.9670   -0.0022    0.2547  274.9902
         0         0         0    1.0000];
T(:,:,2)=[    0.2679   -0.0021   -0.9635  -38.6164
    0.0070    1.0004   -0.0020    0.3990
    0.9680   -0.0005    0.2674  275.3708
         0         0         0    1.0000];
T(:,:,3)=[    0.8727    0.0035    0.4882   17.2597
   -0.0009    1.0000   -0.0056    0.0313
   -0.4883    0.0044    0.8727  296.1788
         0         0         0    1.0000];
T(:,:,4)=[    0.9668    0.0104   -0.2554   14.5343
   -0.0091    0.9999    0.0062   -0.8658
    0.2554   -0.0037    0.9668  292.6855
         0         0         0    1.0000];
T(:,:,5)=[    0.4951    0.0232    0.8685   53.6266
   -0.0118    0.9997   -0.0200   -0.3790
   -0.8687   -0.0004    0.4953  272.5415
         0         0         0    1.0000];
T(:,:,6)=[    -0.2547    0.0406    0.9662   59.0845
    0.0127    0.9992   -0.0386   -0.0847
   -0.9670    0.0024   -0.2550  243.7134
         0         0         0    1.0000];
T(:,:,7)=[     0.8609    0.0176    0.5085   -7.1824
   -0.0188    0.9998   -0.0027   -0.2079
   -0.5085   -0.0072    0.8610  290.3278
         0         0         0    1.0000];
T(:,:,8)=[     0.7177    0.0029   -0.6963  -48.5086
   -0.0065    1.0000   -0.0025   -0.9463
    0.6963    0.0063    0.7177  279.9733
         0         0         0    1.0000];
T(:,:,9)=[     0.0053   -0.0119   -0.9999  -60.3080
   -0.0042    0.9999   -0.0119   -0.3848
    1.0000    0.0043    0.0053  253.5336
         0         0         0    1.0000];
end