
tic;
Cf=[
0 0 0 0
0 0 15 0
15 0 0 0
0 0 15 180
15 180 0 0

15 0 15 180
30 0 30 180
45 0 45 180

15 180 15 0
30 180 30 0
45 180 45 0

15 90 15 -90
30 90 30 -90
45 90 45 -90

15 -90 15 90
30 -90 30 90
45 -90 45 -90

0 0 15 90
45 0 30 180
45 0 60 180
0 0 0 0
    
];

% Cf(90,:)=[];Cf(86,:)=[];Cf(85,:)=[];Cf(80,:)=[];Cf(53,:)=[];
% Cf(6,:)=[];
z=zeros(size(Cf,1),2);
temC=Cf*pi/180;
Cf=[z temC];
M=[
    1.17916 11.5462 85.9337    -3.81232 10.8218 84.9382    -8.78279 10.1976 84.7625    1.75548 6.4901 85.0359    -3.18936 5.83202 84.4369    -8.12148 5.21509 84.2611    2.33147 1.52878 84.5234    -2.57576 0.909814 84.2066    -7.44404 0.293179 83.1065
   1.17475 11.5187 85.7842    -3.81886 10.8285 85.1552    -8.82569 10.2261 85.1439    1.74074 6.46691 84.9858    -3.2014 5.8585 84.7398    -8.10527 5.20387 83.9723    2.33068 1.52213 84.5268    -2.59018 0.893311 83.9389    -7.43551 0.26605 82.9944
   1.17377 11.5194 85.8354    -3.81874 10.8182 85.1356    -8.8077 10.2297 85.0568    1.75265 6.47436 84.9578    -3.19761 5.8215 84.4258    -8.13747 5.20842 84.3327    2.34496 1.52182 84.521    -2.57891 0.885628 83.7066    -7.43987 0.282706 83.0457
  -31.9618 8.35071 80.8699    -36.5205 7.69402 78.9033    -40.6827 6.98093 76.1551    -30.9484 3.38326 79.8646    -35.7664 2.79771 78.6634    -39.9206 2.14765 75.9779    -30.4394 -1.46935 80.1994    -34.9229 -2.07923 78.2032    -39.3472 -2.67361 76.1319
   -31.866 8.33406 80.6029    -36.5293 7.68611 78.903    -40.7223 6.98211 76.2285    -31.0795 3.40419 80.22    -35.7166 2.79046 78.5351    -39.8842 2.14913 75.9058    -30.3434 -1.45597 79.9234    -34.8219 -2.08305 77.9535    -39.1942 -2.66556 75.8039
   -31.9205 8.34138 80.6975    -36.4655 7.67322 78.6914    -40.7414 6.97747 76.251    -31.1091 3.40468 80.2681    -35.829 2.80091 78.7609    -40.1963 2.16896 76.4853    -30.4695 -1.46652 80.2483    -35.0115 -2.08786 78.3773    -39.2787 -2.6797 75.9663
  -37.1025 8.21784 79.8087    -41.4803 7.54748 77.52    -45.8506 6.87765 75.4227    -36.5674 3.33393 80.1202    -40.7386 2.6736 77.4014    -45.0704 2.03958 75.2352    -35.3704 -1.55067 78.8641    -40.0819 -2.18495 77.4303    -44.1418 -2.75226 74.7881
  -37.1193 8.23689 79.8388    -41.5362 7.53227 77.6554    -45.8847 6.89464 75.4909    -36.6292 3.34479 80.2473    -40.679 2.67217 77.2699    -45.665 2.05471 76.244    -35.4763 -1.55027 79.0763    -39.951 -2.17774 77.1527    -44.2488 -2.74618 74.9688
  -37.1846 8.23391 79.9855    -41.4587 7.52385 77.4702    -45.6152 6.84977 75.0263    -36.4735 3.34636 79.8684    -40.664 2.65554 77.2535    -45.3489 2.04586 75.6983    -35.4834 -1.55025 79.104    -40.1577 -2.18767 77.5745    -44.2154 -2.7599 74.9197
   34.5161 14.5155 81.3742    29.3127 13.834 81.9572    24.3515 13.2356 83.0662    34.6285 9.34007 80.2928    29.5885 8.71045 81.1365    24.6595 8.11449 82.1422    34.811 4.26605 79.3994    29.948 3.67406 80.5094    24.9137 3.08181 81.0802
  34.4917 14.5017 81.2873    29.2098 13.7806 81.6752    24.4238 13.2788 83.2747    34.7083 9.35554 80.4481    29.6875 8.72874 81.3654    24.534 8.07402 81.7282    34.9978 4.30784 79.7449    29.9932 3.67576 80.5876    24.9841 3.10231 81.2948
    34.4407 14.4737 81.1621    29.2129 13.7767 81.6928    24.44 13.2807 83.3261    34.6641 9.33894 80.3505    29.6238 8.70363 81.2042    24.6742 8.10645 82.1619    34.9874 4.28146 79.7247    29.9223 3.66594 80.4296    25.1028 3.09399 81.6483
    40.4725 14.4363 81.4477    34.909 13.6332 81.427    29.7847 12.9881 82.0407    40.2531 9.16678 79.9065    35.4386 8.59871 81.2325    30.3869 7.98289 82.0245    40.4158 4.11777 79.0722    35.403 3.53159 79.8778    30.5706 2.93738 80.9657
    40.6 14.4796 81.6995    35.0726 13.6957 81.7803    29.8123 12.986 82.0826    40.2851 9.16477 79.9499    35.3364 8.5674 80.9921    30.326 7.9624 81.8684    40.9022 4.13666 79.9466    35.5357 3.52057 80.1466    30.7147 2.94042 81.3187
    40.5849 14.4579 81.6656    35.1781 13.7197 81.9957    29.8843 13.0181 82.2534    40.2598 9.17264 79.9014    35.3936 8.58744 81.1132    30.4106 7.97545 82.0729    40.6869 4.13009 79.5418    35.4193 3.51134 79.889    30.6734 2.94878 81.1898
    -4.3337 11.3643 85.9074    -9.31177 10.6756 85.4009    -14.3018 10.1007 85.3172    -3.68241 6.31592 84.8313    -8.60668 5.66194 84.5427    -13.5562 5.0599 84.3841    -3.07705 1.41317 84.5392    -7.91771 0.762826 83.5589    -12.8792 0.132613 83.9482
    -4.33464 11.3704 86.0298    -9.28501 10.6628 85.1354    -14.2767 10.0654 85.0635    -3.681 6.30827 84.7892    -8.60741 5.67838 84.4893    -13.5518 5.05495 84.344    -3.06584 1.40498 84.2375    -7.9206 0.754757 83.6505    -12.7737 0.132319 83.1224
    -4.33081 11.335 85.6675    -9.29775 10.6762 85.2103    -14.3019 10.0994 85.2791    -3.68038 6.32127 84.8912    -8.61486 5.67844 84.5551    -13.499 5.02884 83.9627    -3.07306 1.41137 84.449    -7.94607 0.757736 83.8494    -12.8215 0.143973 83.5744
    -8.47815 11.2698 87.3155    -13.3134 10.5039 86.0608    -18.3663 9.93178 86.4661    -7.77652 6.24882 86.5612    -12.6785 5.5995 85.9203    -17.549 4.94536 85.5358    -7.10771 1.31509 85.7338    -11.9687 0.667539 85.3702    -16.8075 0.0543221 84.914
    -8.44217 11.2117 86.8638    -13.3743 10.5533 86.461    -18.3729 9.94903 86.5135    -7.77045 6.26581 86.6225    -12.7034 5.60439 86.1743    -17.6133 4.96792 85.854    -7.11421 1.31437 85.8909    -11.9925 0.674904 85.703    -16.8085 0.0501026 84.9244
    -8.44023 11.2346 86.937    -13.4751 10.6358 87.2211    -18.4389 9.99138 86.8718    -7.77936 6.24955 86.611    -12.7101 5.61033 86.2207    -17.6404 4.9819 86.0736    -7.12072 1.32047 85.962    -12.0068 0.667208 85.6797    -16.8407 0.0538642 85.0706
    -11.6221 11.3251 89.9225    -16.4447 10.5429 88.7084    -21.5547 9.97306 89.4894    -10.879 6.29676 89.0625    -15.7385 5.58808 88.3575    -20.6955 4.96633 88.5319    -10.1348 1.32295 87.8732    -15.0451 0.677372 88.1258    -19.8868 0.0678949 87.7337
    -11.6092 11.2995 89.7498    -16.5221 10.5852 89.1722    -21.5122 9.94306 89.2544    -10.8505 6.26241 88.8396    -15.7537 5.59 88.4348    -20.6424 4.95278 88.2433    -10.1551 1.34941 88.2179    -15.0568 0.678743 88.147    -19.9193 0.0825338 87.9497
   -11.5688 11.2488 89.2529    -16.4811 10.5266 88.8387    -21.5049 9.93302 89.1493    -10.7823 6.23714 88.1563    -15.8227 5.61986 88.8686    -20.7315 4.97009 88.6144    -10.1537 1.32648 88.0374    -15.0695 0.69147 88.2129    -19.8733 0.0605675 87.6602
    6.55634 11.227 86.5293    1.53676 10.5332 85.6891    -3.41864 9.87033 84.9405    7.11543 6.20596 85.9788    2.125 5.56439 85.4202    -2.81356 4.91986 84.5331    7.60968 1.22947 85.0315    2.68762 0.618246 84.5956    -2.20612 0.0178904 83.9384
    6.56594 11.2356 86.5273    1.55313 10.5819 86.0282    -3.42418 9.94026 85.4832    7.09957 6.19148 85.877    2.1052 5.53 84.8806    -2.8158 4.90243 84.4258    7.62298 1.23276 85.1157    2.68328 0.625478 84.3854    -2.20653 0.0152386 83.6854
    6.57269 11.2559 86.706    1.55527 10.5796 86.1131    -3.41635 9.93523 85.3751    7.11565 6.20692 85.9682    2.11454 5.55921 85.1533    -2.80811 4.91078 84.4058    7.61635 1.22876 85.1156    2.69544 0.625613 84.6063    -2.21159 0.0105676 83.6334
   10.8373 11.1417 88.7786    5.77246 10.4558 87.8548    0.801424 9.816 87.1529    11.3963 6.1234 88.484    6.31177 5.44499 87.0872    1.37993 4.83499 86.4479    11.74 1.12199 86.8741    6.8382 0.511056 86.2518    1.94686 -0.089599 85.7547
    10.8356 11.1471 88.8422    5.81029 10.5031 88.1583    0.806458 9.8352 87.1735    11.3552 6.11204 88.2182    6.28549 5.41009 86.8994    1.40309 4.85323 86.7888    11.7329 1.12964 86.7484    6.86613 0.508336 86.5241    1.92624 -0.104391 85.4031
    10.8242 11.1242 88.724    5.80492 10.4935 88.0705    0.800119 9.79959 87.0503    11.332 6.08753 88.0416    6.3479 5.45308 87.489    1.39809 4.86516 86.8378    11.7892 1.12746 87.1444    6.85086 0.499773 86.4207    1.94207 -0.10103 85.6351
   13.486 10.87 91.6507    8.51165 10.2954 91.2394    3.48656 9.60501 89.9238    13.87 5.82661 90.4065    9.05014 5.26581 90.6875    4.04036 4.61682 89.2261    14.3688 0.894184 89.9482    9.4735 0.306776 89.3175    4.5592 -0.28876 88.2533
   13.5121 10.8965 91.8231    8.50778 10.282 91.2456    3.49075 9.62068 89.8875    14.0594 5.90633 91.4754    9.03267 5.26369 90.4933    4.02942 4.59242 89.0615    14.4433 0.906449 90.2996    9.47689 0.296098 89.2997    4.59648 -0.287863 88.6746
    13.5088 10.8959 91.772    8.48402 10.2678 90.9236    3.51023 9.63109 90.036    13.9997 5.88345 91.1106    9.04746 5.26957 90.5798    4.04119 4.61024 89.0154    14.5003 0.913869 90.5715    9.50652 0.316768 89.451    4.58003 -0.283477 88.2602
    1.16372 1.20095 85.4709    -3.73225 0.581654 84.5242    -8.59256 -0.0286556 84.2084    1.71267 -3.63966 84.5724    -3.13824 -4.2473 84.2143    -7.93977 -4.824 83.2368    2.26741 -8.40894 83.8616    -2.53848 -8.95803 83.0094    -7.32992 -9.58124 82.8698
    1.15087 1.19886 85.2099    -3.72963 0.583784 84.3681    -8.61541 -0.021984 84.495    1.73467 -3.65785 84.8112    -3.12622 -4.24018 84.0521    -7.9692 -4.82252 83.4632    2.27437 -8.41116 83.8849    -2.52442 -8.98033 83.2409    -7.31345 -9.55322 82.6333
    1.14386 1.20314 85.1731    -3.73479 0.574063 84.4204    -8.63012 -0.0264659 84.6464    1.72755 -3.65431 84.9018    -3.12996 -4.23236 84.098    -7.96216 -4.82849 83.4439    2.28991 -8.42251 84.0868    -2.53316 -9.00309 83.3601    -7.31752 -9.55407 82.6724
   1.38656 -5.28743 87.2673    -3.46168 -5.92888 87.2778    -8.26374 -6.48632 86.4682    1.93151 -9.99994 86.432    -2.84824 -10.6446 86.45    -7.61082 -11.1746 85.6627    2.51749 -14.7064 86.2433    -2.24977 -15.2444 85.4401    -6.96751 -15.7997 85.0019
    1.39466 -5.30358 87.4059    -3.44437 -5.87041 86.372    -8.25308 -6.46827 86.29    1.94214 -10.0161 86.57    -2.85228 -10.6793 86.7056    -7.61036 -11.1613 85.5408    2.51529 -14.7214 86.2738    -2.25191 -15.2138 85.3085    -7.00016 -15.8702 85.3676
    1.38529 -5.2881 87.2763    -3.44224 -5.89375 86.6189    -8.27689 -6.51434 86.6863    1.93929 -9.98612 86.4548    -2.84095 -10.6193 86.2795    -7.59887 -11.1541 85.4501    2.49694 -14.6647 85.8591    -2.24534 -15.3156 85.7318    -6.99513 -15.8699 85.3685
    1.50099 -11.5524 91.1655    -3.27784 -12.1561 90.6646    -8.05029 -12.7719 90.47    2.05163 -16.1691 90.4499    -2.67041 -16.9085 90.8895    -7.41188 -17.4373 90.0559    2.62945 -20.6975 89.8399    -2.068 -21.3046 89.3779    -6.76571 -21.9158 89.1841
    1.48631 -11.5292 90.8567    -3.26807 -12.1169 90.3502    -8.05982 -12.8025 90.6532    2.0709 -16.1922 90.6197    -2.66996 -16.8812 90.6093    -7.40289 -17.4176 89.9941    2.65042 -20.7976 90.3081    -2.07121 -21.3006 89.4456    -6.76426 -21.9199 89.1783
    1.5073 -11.5883 91.4412    -3.26817 -12.1362 90.5408    -8.03408 -12.7777 90.378    2.06274 -16.1515 90.3721    -2.66661 -16.7247 89.9161    -7.39339 -17.3658 89.6888    2.63125 -20.7196 89.9297    -2.07392 -21.3191 89.4288    -6.78236 -22.0184 89.5364
    0.0920822 21.1598 88.0189    -4.98396 20.3978 87.316    -10.0191 19.6885 86.7371    0.694128 15.8604 86.8535    -4.31706 15.1356 86.0856    -9.32771 14.5143 85.9252    1.30444 10.796 86.3077    -3.67715 10.1064 85.5674    -8.62515 9.4556 85.0931
    0.0933962 21.103 87.8859    -4.9965 20.399 87.3121    -9.9658 19.5489 86.1027    0.709065 15.9648 87.3014    -4.33098 15.1801 86.3985    -9.31131 14.5034 85.7852    1.28933 10.7774 86.1085    -3.66829 10.1065 85.7979    -8.67138 9.53148 85.7758
    0.0886201 21.2163 88.2975    -4.99618 20.4391 87.4951    -9.96711 19.5429 86.0845    0.706114 15.9612 87.3051    -4.32203 15.1826 86.3342    -9.28795 14.4494 85.6102    1.29323 10.7795 86.122    -3.67872 10.1263 85.7311    -8.66349 9.50611 85.7179
    -0.269259 29.2062 91.3818    -5.40971 28.6586 91.3852    -10.5021 27.8368 90.589    0.37573 24.0227 90.9187    -4.71186 23.2699 90.3531    -9.7633 22.5745 89.8457    0.999062 18.7859 90.0893    -4.04502 18.1614 90.0781    -9.01987 17.3159 88.7619
    -0.260106 29.4382 92.1278    -5.42082 28.8193 91.8649    -10.5281 27.9637 90.987    0.387163 24.0128 90.9056    -4.72926 23.3717 90.691    -9.78729 22.6069 89.9777    1.00052 18.8009 90.1296    -4.03758 18.1407 89.9439    -9.04207 17.3606 88.9494
   -0.271352 29.2471 91.4993    -5.40255 28.5779 91.1612    -10.493 27.9009 90.7729    0.38001 23.9861 90.7802    -4.71443 23.3135 90.46    -9.79356 22.6714 90.2097    1.00253 18.8024 90.1057    -4.04389 18.1794 90.1328    -9.03357 17.3376 88.8134
   -1.08696 38.2027 96.9804    -6.26092 37.6498 97.0298    -11.4275 36.8526 96.3723    -0.401315 33.091 97.0283    -5.55042 32.1028 95.8407    -10.6095 31.2608 95.016    0.244908 27.6275 95.9373    -4.87494 27.0717 96.0595    -9.90157 26.0413 94.5319
   -1.08471 38.5548 97.8093    -6.26804 37.6829 97.1334    -11.419 36.8193 96.2733    -0.417336 32.962 96.7161    -5.55242 32.121 95.8986    -10.6226 31.3082 95.1721    0.265058 27.7181 96.2044    -4.87682 27.099 96.1664    -9.89794 25.9859 94.3409
    -1.08117 38.261 97.0518    -6.26805 37.5755 96.8586    -11.4402 36.9186 96.5511    -0.401675 32.9803 96.7523    -5.53555 32.0144 95.5331    -10.5838 31.1244 94.6303    0.252554 27.6661 96.0291    -4.87379 26.9969 95.7872    -9.94714 26.168 94.9616
    3.42576 -20.2891 78.995    -1.28827 -20.7871 78.3825    -5.98948 -21.3359 78.2434    3.95941 -24.5721 76.8921    -0.71947 -25.0374 76.4158    -5.37787 -25.7201 76.4885    4.46218 -28.6794 74.6441    -0.12856 -29.2636 74.4705    -4.75248 -29.8547 74.2919
  3.43482 -20.2994 78.9885    -1.2864 -20.7602 78.286    -5.98766 -21.3601 78.2763    3.95887 -24.5563 76.8515    -0.705432 -25.1397 76.6398    -5.38231 -25.7437 76.5572    4.46262 -28.7002 74.6932    -0.128157 -29.2899 74.4945    -4.75487 -29.8438 74.25
   3.3992 -20.2412 78.714    -1.28512 -20.8739 78.7204    -5.97504 -21.3141 78.1046    3.94245 -24.4978 76.6589    -0.712426 -25.0907 76.5085    -5.35536 -25.6381 76.2113    4.46657 -28.6862 74.639    -0.122422 -29.2954 74.482    -4.742 -29.7887 74.0871
    -44.5517 7.35942 81.9414    -49.0701 6.70221 80.1654    -54.1197 6.1112 79.3172    -43.7428 2.52269 81.7156    -48.2745 1.86358 79.9764    -53.1988 1.24481 78.9214    -42.94 -2.34084 81.428    -47.6467 -2.9846 80.0705    -52.1844 -3.59297 78.4285
   -44.6952 7.38137 82.2383    -49.015 6.6836 80.074    -53.4736 6.0153 78.3387    -43.8343 2.49969 81.8895    -48.2451 1.86064 79.9033    -52.6881 1.23377 78.1795    -42.9114 -2.33586 81.3851    -47.5806 -2.97256 79.9738    -52.3538 -3.60895 78.6766
    -44.5215 7.35034 81.8732    -48.8131 6.64881 79.7399    -53.8462 6.05291 78.9001    -43.6954 2.50189 81.6176    -48.6022 1.86418 80.5375    -52.8322 1.22689 78.3726    -42.8229 -2.32351 81.1919    -47.499 -2.96861 79.7992    -52.1165 -3.59429 78.3054
    22.6169 13.9059 86.5088    17.7937 13.3306 88.0659    12.9491 12.7365 89.589    22.9754 8.80719 85.5727    18.1887 8.2392 87.0281    13.342 7.60433 88.2687    23.6643 3.87398 85.7885    18.6385 3.2222 86.255    13.7868 2.6161 87.1851
    22.6318 13.9047 86.5503    17.7928 13.3214 88.0315    12.9362 12.7279 89.5339    23.0562 8.82867 85.856    18.2209 8.24608 87.1936    13.3404 7.59595 88.2313    23.6126 3.85644 85.6186    18.706 3.23003 86.543    13.8269 2.60603 87.3905
    22.8096 14.0273 87.2222    17.7632 13.3138 87.9853    12.9199 12.7106 89.4154    22.9677 8.80182 85.5438    18.2239 8.25326 87.2185    13.339 7.58036 88.2503    23.6329 3.86647 85.7088    18.7467 3.22923 86.7282    13.8324 2.60621 87.4138
 1.20568 10.7933 85.6087    -3.79829 10.1084 84.9197    -8.72676 9.46044 84.3516    1.75099 5.70701 84.2568    -3.16976 5.09327 84.1379    -8.05825 4.47127 83.4989    2.32528 0.814459 83.7785    -2.55938 0.192802 83.7077    -7.4414 -0.415335 83.2699
    1.21836 10.8165 85.9651    -3.79991 10.095 84.9096    -8.71133 9.41686 84.084    1.7612 5.72776 84.5653    -3.17195 5.10888 84.238    -8.06438 4.47258 83.5743    2.33661 0.816848 83.8623    -2.56611 0.190816 83.4153    -7.42185 -0.411925 82.9688
    1.21774 10.7937 85.8475    -3.79976 10.1063 84.9992    -8.72535 9.44021 84.2274    1.78467 5.74432 84.8305    -3.16045 5.07996 83.9979    -8.05152 4.4654 83.4608    2.32609 0.808628 83.687    -2.56586 0.18364 83.7239    -7.43238 -0.409441 83.0044 
 
 ];
Mc=zeros(size(M,1)/3,size(M,2));
Tst2cb=zeros(4,4,size(Mc,1));
Tcb2st=zeros(4,4,size(Mc,1));
% gamma3=-0.163;
% gamma3T=[cos(gamma3), -sin(gamma3),0,0;sin(gamma3), cos(gamma3),0,0;0, 0, 1, 0;0, 0, 0, 1];

figure;hold on;xlabel('X');ylabel('Y');zlabel('Z');axis equal;view([45 20]);axis([ -50 50 -50 50 -150 150]);
j=1;
for i = 1 :3: size(M,1)
    Mc(j,:)=mean(M(i:i+2,:));
    j=j+1;
end


for i = 1: size(Mc,1)
    A=Mc(i,1:3);B=Mc(i,4:6);C=Mc(i,7:9);D=Mc(i,10:12);E=Mc(i,13:15);F=Mc(i,16:18);G=Mc(i,19:21);H=Mc(i,22:24);I=Mc(i,25:27);
    nx=(C-A+F-D+I-G)/norm((C-A+F-D+I-G));
    ny=(G-A+H-B+I-C)/norm((G-A+H-B+I-C));
    nz=cross(nx,ny)/norm(cross(nx,ny));
    P=(A+B+C+D+E+F+G+H+I)'/9;
    nx=cross(ny,nz);
    R=[nx',ny',nz'];
    T=[R,P;[0 0 0 1]];
    Tst2cb(:,:,i)=T;
    Tcb2st(:,:,i)=inv(Tst2cb(:,:,i));
    plotCoord(Tst2cb(:,:,i));
    plotCoord(Tcb2st(:,:,i));
end
% plotCoord(Tcb2tc,3);
% % % % % 
% % % % % lineData=Tcb2st(1:3,4,1:17);
% % % % % rz=getZAxis(lineData);
% % % % % 
% % % % % dp=rz/2;dp(3)=rz(3);rz=dp/norm(dp);
% % % % % % rz=[0 0 1]';
% % % % % planeData=Tcb2st(1:3,4,1:32);
% % % % % ry=-getYAxis(planeData);
% % % % % dp=ry/1.2;dp(3)=ry(3);ry=dp/norm(ry);
% % % % % rx=cross(ry,rz)/norm(cross(ry,rz));
% % % % % ry=cross(rz,rx)/norm(cross(rz,rx));
% % % % % R=[rx,ry,rz];
% % % % % 
% % % % % 
% % % % % %%
% % % % % % pointData=Tst2cb(:,:,1:7);
% % % % % % P0=Tst2cb(1:3,4,1);
% % % % % % pointInvData=pointData;
% % % % % % for i = 1:7
% % % % % %     pointData(1:3,4,i)=P0;
% % % % % %     pointInvData(:,:,i)=inv(pointData(:,:,i));
% % % % % % end
% % % % % % pData=pointInvData(1:3,4,:);
% % % % % % pData=reshape(pData,[3 7])';
% % % % % % % options = optimoptions('fmincon','Display','iter','Algorithm','interior-point'); 
% % % % % % [x,y]=fmincon('getInitialPoint',P0',[],[],[],[],[],[],[]);
% % % % % 
% % % % % 
% % % % % Pinitial=reshape(Tcb2st(1:3,4,15:17),[3 3])';
% % % % % P=mean(Pinitial)';
% % % % % % P=[15.1034    7.8430  -97.9690]';
% % % % % Tcb2tc=[R,P;[0 0 0 1]];
% % % % % % plotCoord(Tcb2tc,3);
% % % % % % plotCoord(inv(Tcb2tc),3);
% % % % % Ttc2st=zeros(4,4,size(M,1));
% % % % % axang=zeros(size(M,1),4);
% % % % % figure;hold on;xlabel('X');ylabel('Y');zlabel('Z');axis equal;view([45 20]);axis([ -50 50 -50 50 50 150]);
% % % % % title("asdsad");
% % % % % for i = 1:size(M,1)
% % % % %     Ttc2st(:,:,i)=inv(Tcb2tc)*Tcb2st(:,:,i);
% % % % %     Ttc2st(3,4,i)=Ttc2st(3,4,i)+75;
% % % % %     
% % % % % end
% % % % % plotCoord(inv(Tcb2tc),3);
% % % % % 
% % % % % 
% % % % % 
% % % % % 
% % % % % sGamma3=1:17;
% % % % % x=asin(-Ttc2st(1,2,sGamma3));
% % % % % y=asin(Ttc2st(2,3,sGamma3));
% % % % % unknownerror=mean(y);
% % % % % gamma3=mean(x);
% % % % % gamma3T=[cos(gamma3), -sin(gamma3),0,0;sin(gamma3), cos(gamma3),0,0;0, 0, 1, 0;0, 0, 0, 1];
% % % % % uke3T=[1 0 0 0;0 cos(gamma3), sin(gamma3),0;0 -sin(gamma3), cos(gamma3),0;0, 0, 0, 1];
% % % % % Ttc2end=zeros(4,4,size(M,1));

fileConfig = fopen('T.txt','w');
filePsi = fopen('psi.txt','w');
for i = 1:size(Mc,1)
% % %     Ttc2end(:,:,i)=Ttc2st(:,:,i)*inv(gamma3T);%*inv(uke3T);
%     plotCoord(Ttc2end(:,:,i));
    
    fprintf(fileConfig,'%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n',Tst2cb(:,:,i));
    fprintf(filePsi,'%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n',Cf(i,:));

end
fclose(fileConfig);
fclose(filePsi);
toc; 