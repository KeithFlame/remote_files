function [T,Config]=getTandConfig
T=zeros(4,4,34);
T(:,:,1) =[1.0000   -0.0002    0.0098   -2.7874
    0.0001    0.9999    0.0103   -2.8816
   -0.0098   -0.0103    0.9999   74.9424
         0         0         0    1.0000
];

T(:,:,2) =[ 0.9996    0.0030   -0.0274    0.9034
   -0.0031    1.0000   -0.0031   -2.0715
    0.0274    0.0032    0.9996   79.6498
         0         0         0    1.0000

];
T(:,:,3) =[0.9998    0.0026   -0.0203    0.2694
   -0.0029    0.9998   -0.0178   -1.2934
    0.0202    0.0178    0.9996   84.6823
         0         0         0    1.0000
];

T(:,:,4) =[0.9999    0.0019   -0.0169   -0.0582
   -0.0022    0.9998   -0.0177   -1.9084
    0.0169    0.0177    0.9997   89.5852
         0         0         0    1.0000

];
T(:,:,5) =[0.9999    0.0022   -0.0117   -0.5096
   -0.0025    0.9996   -0.0274   -1.7414
    0.0116    0.0274    0.9996   94.6014
         0         0         0    1.0000

];
T(:,:,6) =[0.9998    0.0037   -0.0199    0.1297
   -0.0046    0.9991   -0.0424   -1.2836
    0.0198    0.0425    0.9989   99.5590
         0         0         0    1.0000
];

T(:,:,7) =[0.9994    0.0051   -0.0338    1.0785
   -0.0065    0.9991   -0.0411   -2.1130
    0.0335    0.0413    0.9986  104.5349
         0         0         0    1.0000

];
T(:,:,8) =[0.9999    0.0009   -0.0157   -9.3032
   -0.0010    1.0000   -0.0078   -2.0392
    0.0157    0.0079    0.9998   94.0834
         0         0         0    1.0000
];

T(:,:,9) =[0.9999   -0.0013   -0.0156  -15.9176
    0.0011    0.9999   -0.0134   -0.6318
    0.0156    0.0134    0.9998   91.9504
         0         0         0    1.0000
];

T(:,:,10) =[0.9999   -0.0031   -0.0127  -22.0088
    0.0030    0.9999   -0.0116    0.3647
    0.0128    0.0115    0.9999   88.3227
         0         0         0    1.0000

];
T(:,:,11) =[0.9999    0.0067   -0.0152    8.9045
   -0.0074    0.9990   -0.0441   -1.1766
    0.0149    0.0442    0.9989   94.4971
         0         0         0    1.0000
];

T(:,:,12) =[0.9999    0.0095   -0.0075   14.8119
   -0.0099    0.9986   -0.0519   -1.1590
    0.0070    0.0520    0.9986   92.2462
         0         0         0    1.0000
];

T(:,:,13) =[1.0000    0.0092   -0.0019   20.3588
   -0.0093    0.9976   -0.0691    0.4523
    0.0013    0.0692    0.9976   89.1862
         0         0         0    1.0000
];

T(:,:,14) =[0.9997    0.0029   -0.0252    0.6224
   -0.0032    0.9999   -0.0126    6.5879
    0.0251    0.0127    0.9996   96.0574
         0         0         0    1.0000
];

T(:,:,15) =[0.9998    0.0060   -0.0185    0.4741
   -0.0063    0.9998   -0.0164   13.9610
    0.0184    0.0165    0.9997   95.4429
         0         0         0    1.0000
];

T(:,:,16) =[0.9993    0.0152   -0.0354    2.5565
   -0.0158    0.9998   -0.0155   20.4748
    0.0351    0.0161    0.9993   93.3610
         0         0         0    1.0000

];
T(:,:,17) =[0.9970   -0.0133    0.0759  -18.8386
    0.0146    0.9998   -0.0163    0.0268
   -0.0756    0.0173    0.9970   89.0059
         0         0         0    1.0000
];

T(:,:,18) =[0.9881   -0.0232    0.1521  -14.6127
    0.0240    0.9997   -0.0038   -1.2081
   -0.1520    0.0074    0.9884   88.9151
         0         0         0    1.0000

];
T(:,:,19) =[0.9741   -0.0282    0.2242  -10.1082
    0.0342    0.9992   -0.0227    0.2388
   -0.2234    0.0297    0.9743   88.8064
         0         0         0    1.0000
];

T(:,:,20) =[0.9408   -0.0350    0.3372   -9.9584
    0.0517    0.9978   -0.0407    1.8994
   -0.3350    0.0558    0.9406   88.9346
         0         0         0    1.0000
];

T(:,:,21) =[0.8884   -0.0372    0.4575  -10.8312
    0.0830    0.9933   -0.0805    5.6776
   -0.4515    0.1095    0.8856   89.5571
         0         0         0    1.0000
];

T(:,:,22) =[0.9906   -0.0069    0.1366   17.7560
    0.0262    0.9897   -0.1404    6.4496
   -0.1343    0.1427    0.9806   88.7136
         0         0         0    1.0000

];
T(:,:,23) =[0.9908    0.0277   -0.1324   20.5897
   -0.0344    0.9982   -0.0488   -1.5900
    0.1308    0.0529    0.9900   90.2559
         0         0         0    1.0000
];

T(:,:,24) =[0.9796    0.0347   -0.1978   15.7745
   -0.0422    0.9985   -0.0340   -2.2828
    0.1963    0.0417    0.9797   89.7900
         0         0         0    1.0000
];

T(:,:,25) =[0.9608    0.0431   -0.2738   11.8868
   -0.0548    0.9979   -0.0352   -1.5784
    0.2717    0.0488    0.9612   89.5414
         0         0         0    1.0000
];

T(:,:,26) =[0.9997    0.0035   -0.0242    0.8543
   -0.0044    0.9994   -0.0352   -1.7160
    0.0241    0.0353    0.9991   95.1108
         0         0         0    1.0000

];
T(:,:,27) =[0.9900   -0.0066    0.1407   17.1243
    0.0259    0.9904   -0.1356    5.9714
   -0.1384    0.1379    0.9807   88.4542
         0         0         0    1.0000

];
T(:,:,28) =[0.9905    0.0289   -0.1346   20.4609
   -0.0347    0.9986   -0.0408   -2.3223
    0.1333    0.0450    0.9901   90.2193
         0         0         0    1.0000
];

T(:,:,29) =[0.9768    0.0350   -0.2115   16.7331
   -0.0383    0.9992   -0.0117   -4.2317
    0.2109    0.0195    0.9773   90.1404
         0         0         0    1.0000
];

T(:,:,30) =[0.9604    0.0436   -0.2752   11.7716
   -0.0555    0.9978   -0.0356   -1.4530
    0.2731    0.0495    0.9607   89.3203
         0         0         0    1.0000

];
T(:,:,31) =[0.9341    0.0516   -0.3533    8.5440
   -0.0621    0.9979   -0.0184   -2.2759
    0.3516    0.0391    0.9353   88.8394
         0         0         0    1.0000

];
T(:,:,32) =[0.9111    0.0588   -0.4080    3.5687
   -0.0678    0.9977   -0.0078   -2.5170
    0.4066    0.0348    0.9129   87.8215
         0         0         0    1.0000
];

T(:,:,33) =[ 0.9995    0.0016   -0.0307    0.8538
   -0.0020    0.9999   -0.0154   -3.0275
    0.0307    0.0154    0.9994   95.1653
         0         0         0    1.0000
];

T(:,:,34) =[ 0.9997    0.0028   -0.0251    0.4636
   -0.0032    0.9999   -0.0164   -0.5574
    0.0251    0.0165    0.9995   75.5977
         0         0         0    1.0000
         ];

Cf=[
         0         0         0         0         0         0
         0    5.0000         0         0         0         0
         0   10.0000         0         0         0         0
         0   15.0000         0         0         0         0
         0   20.0000         0         0         0         0
         0   25.0000         0         0         0         0
         0   30.0000         0         0         0         0
         0   20.0000    0.2618         0    0.2618    3.1416
         0   20.0000    0.5236         0    0.5236    3.1416
         0   20.0000    0.7854         0    0.7854    3.1416
         0   20.0000    0.2618    3.1416    0.2618         0
         0   20.0000    0.5236    3.1416    0.5236         0
         0   20.0000    0.7854    3.1416    0.7854         0
         0   20.0000    0.2618   -1.5708    0.2618    1.5708
         0   20.0000    0.5236   -1.5708    0.5236    1.5708
         0   20.0000    0.7854   -1.5708    0.7854    1.5708
         0   20.0000    0.7854         0    0.8727    3.1416
         0   20.0000    0.7854         0    0.9599    3.1416
         0   20.0000    0.7854         0    1.0472    3.1416
         0   20.0000    0.7854         0    1.1345    3.1416
         0   20.0000    0.7854         0    1.2217    3.1416
         0   20.0000    0.7854    3.1416    0.6981         0
         0   20.0000    0.7854    3.1416    0.8727         0
         0   20.0000    0.7854    3.1416    0.9599         0
         0   20.0000    0.7854    3.1416    1.0472         0
         0   20.0000         0         0         0         0
         0   20.0000    0.7854    3.1416    0.6981         0
         0   20.0000    0.7854    3.1416    0.8727         0
         0   20.0000    0.7854    3.1416    0.9599         0
         0   20.0000    0.7854    3.1416    1.0472         0
         0   20.0000    0.7854    3.1416    1.1345         0
         0   20.0000    0.7854    3.1416    1.2217         0
         0   20.0000         0         0         0         0
         0         0         0         0         0         0

];
Config=Cf;
% Config(:,2)=Cf(:,2);
% Config(17,:)=[];
end