
tic;
cfg_listfilename = join([getenv('VSARMCALIBPATH'), '\conf\', 'cfg_list.log']);
cfg_raw = load(cfg_listfilename);
Cf = cfg_raw;
% Cf=[
% 0 0 0 0
% 0 0 15 0
% 15 0 0 0
% 0 0 15 180
% 15 180 0 0
% 15 0 15 180
% 30 0 30 180
% 45 0 45 180
% 15 180 15 0
% 30 180 30 0
% 45 180 45 0
% 15 90 15 -90
% 30 90 30 -90
% 45 90 45 -90
% 15 -90 15 90
% 30 -90 30 90
% 45 -90 45 90
% 0 0 15 90
% 45 0 50 180
% 45 0 60 180
% ];

% Cf(90,:)=[];Cf(86,:)=[];Cf(85,:)=[];Cf(80,:)=[];Cf(53,:)=[];
% Cf(6,:)=[];
z=zeros(size(Cf,1),2);
temC=Cf*pi/180;
Cf=[z temC];

outfilename = join([getenv('VSARMCALIBPATH'), '\calib_pic_coord_out.log']);
M_raw = importdata(outfilename);
M = str_rep(M_raw);
% M=[
% [-10.1859, 1.65916, 85.2026]     [-6.56462, 3.79845, 84.5368]    [-2.98322, 5.89928, 83.786]    [-12.3062, 5.19079, 84.6]    [-8.79412, 7.40987, 84.9642]    [-5.18504, 9.55269, 84.4839]    [-14.5874, 8.87011, 85.1135]    [-10.8914, 10.9393, 84.1047]    [-7.32889, 13.039, 83.8498]
% [-10.1692, 1.65414, 84.9145]     [-6.52463, 3.77189, 84.0636]    [-3.03703, 6.03363, 85.0853]    [-12.2356, 5.16333, 83.8607]    [-8.71716, 7.38051, 84.3075]    [-5.19233, 9.5627, 84.5616]    [-14.3814, 8.72151, 83.645]    [-10.9026, 10.8604, 84.0506]    [-7.28412, 12.9193, 83.0068]
% [-10.1728, 1.65561, 85.0673]     [-6.55789, 3.80559, 84.4712]    [-3.0086, 5.99233, 84.9365]    [-12.2508, 5.20421, 84.0823]    [-8.76251, 7.40497, 84.7777]    [-5.13656, 9.44839, 83.4582]    [-14.2457, 8.63759, 82.9459]    [-10.9044, 10.9062, 84.1074]    [-7.27929, 12.9378, 83.1605]
% [-37.5573, 1.76028, 77.1658]     [-34.3419, 3.97799, 78.7447]    [-30.5372, 6.07255, 79.0311]    [-39.3118, 5.25887, 75.7569]    [-36.7901, 7.67734, 78.7612]    [-32.2906, 9.60634, 77.4733]    [-41.6125, 8.84108, 75.3823]    [-37.8783, 10.9596, 75.8176]    [-34.319, 13.1205, 76.5444]
% [-37.7062, 1.73623, 77.5194]     [-33.8882, 3.89281, 77.7504]    [-30.0251, 6.01557, 77.6814]    [-39.85, 5.35808, 76.7937]    [-36.3547, 7.52612, 77.7627]    [-32.4096, 9.60098, 77.6916]    [-41.3581, 8.801, 74.8946]    [-38.3467, 11.0542, 76.6969]    [-34.4768, 13.135, 76.8807]
% [-37.4736, 1.75402, 77.0755]     [-33.9456, 3.90846, 77.8254]    [-30.3709, 6.05901, 78.5836]    [-40.1419, 5.3899, 77.3392]    [-36.3861, 7.56587, 77.8474]    [-32.692, 9.7257, 78.3933]    [-41.3984, 8.75891, 74.9045]    [-38.1182, 11.0124, 76.2364]    [-35.1292, 13.3878, 78.3476]
% [-45.1857, 1.97636, 76.6133]     [-41.1397, 4.1789, 76.5581]    [-37.7378, 6.35673, 77.6274]    [-47.6283, 5.65523, 76.3358]    [-43.7387, 7.80518, 76.7166]    [-40.0177, 9.9661, 77.2567]    [-50.0002, 9.32001, 76.044]    [-45.8129, 11.3815, 75.8678]    [-41.0179, 13.1973, 74.3613]
% [-45.3833, 2.00754, 76.9191]     [-41.4978, 4.20238, 77.2346]    [-37.7412, 6.39232, 77.7046]    [-47.4372, 5.66362, 76.0121]    [-43.1126, 7.71119, 75.5706]    [-39.6017, 9.86585, 76.4057]    [-49.4344, 9.22225, 75.1457]    [-46.0918, 11.4717, 76.2997]    [-41.6983, 13.4143, 75.6365]
% [-45.7262, 2.01746, 77.5885]     [-40.7948, 4.10607, 75.9034]    [-37.6609, 6.35909, 77.548]    [-47.3742, 5.58919, 75.9162]    [-43.4026, 7.7433, 76.0678]    [-39.8873, 9.93507, 76.9977]    [-50.5247, 9.43016, 76.7958]    [-45.2806, 11.2938, 74.9215]    [-42.4595, 13.6951, 77.0481]
% [17.2064, 0.836641, 82.3054]     [20.4138, 2.93017, 80.688]    [23.8045, 5.08319, 79.7118]    [15.0106, 4.32974, 82.3626]    [18.6231, 6.54267, 82.4548]    [21.6188, 8.54199, 79.7535]    [13.135, 7.96389, 84.0893]    [16.1545, 9.89532, 80.9277]    [19.653, 12.0642, 80.5683]
% [17.1125, 0.853375, 81.8854]     [20.6172, 2.97416, 81.3572]    [23.8384, 5.05911, 79.6433]    [15.193, 4.3917, 83.1486]    [18.3888, 6.44009, 81.2693]    [21.5729, 8.48761, 79.4566]    [13.0116, 7.85465, 83.0072]    [16.4647, 10.0244, 82.4822]    [19.4807, 11.936, 79.8388]
% [17.1944, 0.836438, 82.2006]     [20.66, 2.97873, 81.4638]    [23.6356, 5.0227, 78.9635]    [15.0362, 4.31376, 82.2832]    [18.4988, 6.51539, 81.5627]    [21.8809, 8.63224, 80.5427]    [13.0683, 7.90443, 83.3687]    [16.3665, 9.96157, 81.8657]    [19.7585, 12.1139, 80.8169]
% [23.5101, 0.57762, 80.3013]     [26.7982, 2.67831, 79.0628]    [29.9795, 4.71773, 77.5256]    [21.6807, 4.11557, 81.7021]    [24.7481, 6.15083, 79.5877]    [27.7286, 8.16064, 77.5307]    [19.342, 7.50125, 80.9987]    [22.758, 9.65152, 80.3028]    [26.1321, 11.7684, 79.3448]
% [23.7377, 0.587512, 80.8938]     [26.7032, 2.67252, 78.6147]    [30.344, 4.81529, 78.4016]    [21.5891, 4.0771, 81.1307]    [24.7686, 6.1693, 79.6352]    [27.9428, 8.21188, 77.9916]    [19.521, 7.58022, 81.6542]    [22.7788, 9.61639, 80.2664]    [26.0719, 11.735, 79.0839]
% [23.7271, 0.602784, 80.8385]     [26.9449, 2.69331, 79.3343]    [29.7273, 4.66794, 76.6894]    [21.7603, 4.13596, 81.7825]    [24.9553, 6.18941, 80.127]    [28.133, 8.27231, 78.4965]    [19.6065, 7.61967, 82.0609]    [23.1845, 9.80687, 81.5095]    [26.2744, 11.8445, 79.6582]
% [-17.7381, 1.68136, 82.451]     [-14.3794, 3.83747, 83.2828]    [-10.8842, 5.97376, 83.2545]    [-20.2478, 5.22442, 83.9156]    [-16.6071, 7.37058, 83.4562]    [-12.8812, 9.41452, 82.2328]    [-22.0462, 8.66842, 82.3966]    [-18.591, 10.7675, 82.4453]    [-15.1613, 12.9608, 82.5124]
% [-17.8018, 1.68721, 82.7403]     [-14.3268, 3.80162, 82.7283]    [-11.0014, 6.02619, 83.8362]    [-20.0218, 5.20607, 82.9698]    [-16.5687, 7.32617, 83.1899]    [-13.028, 9.44303, 82.8804]    [-22.3235, 8.74676, 83.3124]    [-18.5733, 10.7578, 82.308]    [-15.2285, 13.0074, 83.1029]
% [-18.07, 1.70696, 83.8768]     [-14.2276, 3.77788, 82.1031]    [-11.046, 6.02378, 84.2786]    [-19.8768, 5.17924, 82.3597]    [-16.6095, 7.35643, 83.3976]    [-13.105, 9.50485, 83.3237]    [-22.1307, 8.65836, 82.5882]    [-18.7562, 10.8402, 83.0675]    [-15.0782, 12.8765, 82.1132]
% [-24.626, 1.97224, 84.6339]     [-21.077, 4.07368, 84.5522]    [-17.5095, 6.1732, 84.2047]    [-26.4827, 5.41749, 83.5945]    [-22.7754, 7.44703, 82.6877]    [-19.4084, 9.54347, 82.9876]    [-28.7709, 8.92264, 83.8368]    [-25.3767, 11.0427, 84.092]    [-21.6053, 13.027, 83.0496]
% [-24.5432, 1.996, 84.3807]     [-21.1353, 4.0978, 84.7959]    [-17.4908, 6.17319, 84.1115]    [-26.8894, 5.4484, 84.7779]    [-23.1725, 7.5801, 84.106]    [-19.5854, 9.62662, 83.7343]    [-28.7288, 8.88407, 83.6925]    [-24.98, 10.8944, 82.8427]    [-21.8381, 13.1811, 83.9063]
% [-24.6683, 1.98868, 84.7479]     [-20.9375, 4.02622, 83.9083]    [-17.678, 6.26908, 84.9346]    [-26.8677, 5.46355, 84.8087]    [-23.0506, 7.4978, 83.5137]    [-19.6481, 9.65469, 83.8813]    [-28.6565, 8.91298, 83.527]    [-25.5664, 11.1141, 84.6109]    [-21.7925, 13.1926, 83.8072]
% [-30.9903, 2.02603, 86.2992]     [-27.5241, 4.16826, 86.3728]    [-24.2186, 6.3457, 86.9921]    [-33.3094, 5.47096, 86.5246]    [-29.4641, 7.56916, 85.6313]    [-25.6986, 9.54694, 84.4429]    [-34.7361, 8.79888, 84.5717]    [-30.9607, 10.7815, 83.6121]    [-28.4658, 13.2458, 86.3326]
% [-31.1838, 2.05979, 86.7403]     [-27.7342, 4.16663, 86.9048]    [-24.1628, 6.29464, 86.528]    [-33.2211, 5.49482, 86.2402]    [-29.2529, 7.50981, 84.8831]    [-25.8964, 9.60317, 85.1207]    [-35.5167, 9.0337, 86.477]    [-31.9339, 11.1116, 86.2173]    [-28.1276, 13.0726, 85.1299]
% [-31.1249, 2.0657, 86.5502]     [-27.581, 4.12862, 86.4158]    [-23.8374, 6.1735, 85.4322]    [-33.2367, 5.51974, 86.3048]    [-29.3366, 7.5514, 85.1214]    [-26.3136, 9.76893, 86.3828]    [-34.9854, 8.88652, 85.0679]    [-31.4887, 10.9551, 84.9748]    [-28.1804, 13.099, 85.3895]
% [-2.57476, 0.986177, 84.2217]     [0.947942, 3.11003, 84.6735]    [4.44469, 5.20555, 83.7265]    [-4.64945, 4.39472, 82.8656]    [-1.213, 6.5384, 83.6173]    [2.30436, 8.67587, 83.1989]    [-6.86587, 8.04889, 84.2602]    [-3.29006, 10.01, 82.7969]    [0.173358, 12.2199, 83.3772]
% [-2.58083, 0.981852, 84.853]     [0.938435, 3.07522, 83.8665]    [4.41628, 5.16737, 82.9396]    [-4.67984, 4.4901, 84.0216]    [-1.16464, 6.58086, 83.8566]    [2.35435, 8.74574, 83.6808]    [-6.85604, 8.01798, 84.2911]    [-3.32442, 10.1436, 84.204]    [0.179305, 12.1524, 83.0414]
% [-2.52379, 0.961449, 83.6264]     [0.958762, 3.08704, 84.1208]    [4.45369, 5.16468, 83.3316]    [-4.64375, 4.44961, 83.1821]    [-1.17069, 6.52098, 83.0533]    [2.33418, 8.70119, 83.0302]    [-6.70346, 7.83941, 82.4812]    [-3.28586, 9.96463, 82.8251]    [0.194492, 12.1894, 83.2506]
% [4.22749, 0.467026, 85.7192]     [7.70358, 2.5453, 84.99]    [10.9541, 4.57332, 83.2056]    [2.13284, 3.94683, 85.2962]    [5.53801, 5.99667, 84.475]    [9.06151, 8.16279, 84.7657]    [0.0223262, 7.50186, 85.7563]    [3.43611, 9.45479, 84.3592]    [6.95111, 11.6026, 84.5306]
% [4.32362, 0.47082, 86.2466]     [7.7056, 2.56298, 84.6979]    [11.1122, 4.67888, 84.1735]    [2.12534, 3.91488, 85.3894]    [5.6345, 6.04347, 85.3668]    [8.91958, 8.05965, 83.198]    [0.0184882, 7.42542, 84.9568]    [3.484, 9.50001, 84.7978]    [6.95051, 11.6329, 84.6021]
% [4.27082, 0.435703, 84.9155]     [7.7015, 2.5535, 84.8407]    [11.1045, 4.56938, 84.0041]    [2.13404, 3.90107, 84.6116]    [5.63056, 6.05552, 85.0893]    [8.93003, 8.00627, 83.4305]    [0.0162594, 7.33904, 84.3521]    [3.49381, 9.48874, 85.0588]    [6.91786, 11.5527, 84.2318]
% [10.5912, -0.181486, 87.7725]     [13.944, 1.92282, 86.7014]    [17.2744, 3.93285, 85.9175]    [8.46657, 3.26109, 87.5808]    [11.9454, 5.41348, 87.5912]    [15.2429, 7.36851, 86.2251]    [6.33735, 6.65149, 86.9756]    [9.77138, 8.72817, 86.4971]    [13.1821, 10.81, 86.2531]
% [10.6164, -0.183986, 87.1868]     [14.134, 1.93772, 87.9494]    [17.3432, 3.97751, 86.0402]    [8.49581, 3.2252, 87.1983]    [11.8202, 5.30993, 86.3783]    [15.4321, 7.49166, 87.0473]    [6.41374, 6.74003, 87.6149]    [9.8053, 8.73992, 86.6727]    [13.0031, 10.6249, 85.1327]
% [10.8595, -0.204132, 89.0766]     [14.0098, 1.90582, 86.7951]    [17.6017, 4.05298, 87.3322]    [8.60829, 3.27531, 88.0848]    [12.0379, 5.41854, 87.8073]    [15.4756, 7.50132, 87.2956]    [6.41208, 6.64193, 87.2777]    [9.93008, 8.78735, 87.6178]    [13.3024, 10.9081, 87.0194]
% [-10.7189, -6.31387, 83.5042]     [-7.2662, -4.18373, 83.5903]    [-3.74054, -2.00547, 82.9601]    [-12.7639, -2.80215, 82.8443]    [-9.22908, -0.683051, 82.1373]    [-5.86695, 1.50965, 83.2447]    [-14.7986, 0.660628, 82.0783]    [-11.6025, 2.837, 84.0422]    [-7.92412, 4.89505, 82.5934]
% [-10.6472, -6.31375, 82.9975]     [-7.16974, -4.15751, 82.9242]    [-3.77632, -2.04888, 83.8353]    [-12.7785, -2.78811, 82.8034]    [-9.37105, -0.694182, 83.3598]    [-5.87053, 1.44734, 83.3064]    [-14.9974, 0.691356, 83.1606]    [-11.4092, 2.77912, 82.5928]    [-8.03882, 4.93425, 83.2773]
% [-10.7595, -6.33951, 83.7576]     [-7.22221, -4.20073, 82.9345]    [-3.7373, -2.07338, 83.8099]    [-12.9342, -2.82388, 83.7771]    [-9.38587, -0.690808, 83.5319]    [-5.89364, 1.49464, 83.5516]    [-15.0343, 0.681889, 83.6008]    [-11.4231, 2.76459, 82.462]    [-8.06892, 5.00218, 83.7637]
% [-10.4625, -12.6256, 83.8484]     [-7.1089, -10.5327, 84.265]    [-3.68092, -8.46063, 84.6926]    [-12.7666, -9.34093, 84.8918]    [-9.30587, -7.15781, 84.9273]    [-5.81832, -4.97697, 84.48]    [-15.0399, -5.90976, 85.5254]    [-11.3951, -3.65273, 84.2899]    [-8.12686, -1.54826, 86.1587]
% [-10.4625, -12.6256, 83.8484]     [-7.1089, -10.5327, 84.265]    [-3.68092, -8.46063, 84.6926]    [-12.7666, -9.34093, 84.8918]    [-9.30587, -7.15781, 84.9273]    [-5.81832, -4.97697, 84.48]    [-15.0399, -5.90976, 85.5254]    [-11.3951, -3.65273, 84.2899]    [-8.12686, -1.54826, 86.1587]
% [-10.6445, -12.7762, 85.0661]     [-7.14472, -10.6022, 84.5959]    [-3.77279, -8.76276, 87.6708]    [-12.8503, -9.37694, 85.3034]    [-9.36044, -7.2035, 85.3049]    [-5.82466, -4.93149, 84.3451]    [-14.8872, -5.81865, 84.586]    [-11.4863, -3.69797, 85.0932]    [-7.94584, -1.54915, 84.2648]
% [-10.5957, -20.1147, 88.9678]     [-6.98895, -17.6856, 87.6911]    [-3.6491, -15.9245, 90.4423]    [-12.5656, -16.3143, 87.3351]    [-9.2996, -14.5076, 89.6509]    [-5.64637, -11.8335, 86.8656]    [-14.8923, -13.002, 88.4279]    [-11.2416, -10.6787, 87.0203]    [-7.8787, -8.57868, 87.856]
% [-10.8195, -20.5334, 90.8929]     [-7.01958, -17.7412, 88.0371]    [-3.54838, -15.545, 87.9792]    [-12.8436, -16.6439, 89.1266]    [-9.06826, -14.0603, 86.764]    [-5.68209, -11.9433, 87.3397]    [-14.7943, -12.931, 87.8747]    [-11.398, -10.7571, 87.9865]    [-7.81837, -8.51337, 87.1359]
% [-10.8195, -20.5334, 90.8929]     [-7.01958, -17.7412, 88.0371]    [-3.54838, -15.545, 87.9792]    [-12.8436, -16.6439, 89.1266]    [-9.06826, -14.0603, 86.764]    [-5.68209, -11.9433, 87.3397]    [-14.7943, -12.931, 87.8747]    [-11.398, -10.7571, 87.9865]    [-7.81837, -8.51337, 87.1359]
% [-10.0151, 9.50507, 81.4859]     [-6.74088, 11.8265, 83.239]    [-3.23008, 13.9181, 83.0652]    [-12.2697, 13.1778, 82.6145]    [-8.70027, 15.005, 81.2399]    [-5.26097, 17.1336, 81.21]    [-14.5191, 16.8332, 83.0972]    [-10.7858, 18.5062, 81.1907]    [-7.32271, 20.5464, 80.8817]
% [-10.1098, 9.60205, 82.43]     [-6.69827, 11.74, 82.5008]    [-3.21253, 13.8028, 82.1386]    [-12.2873, 13.0957, 82.4011]    [-8.73465, 15.1347, 81.7497]    [-5.29098, 17.1852, 81.7086]    [-14.5259, 16.8109, 83.1061]    [-10.8372, 18.5709, 81.5802]    [-7.42623, 20.7437, 81.7035]
% [-10.2079, 9.70284, 83.1194]     [-6.7786, 11.8661, 83.4831]    [-3.23594, 13.9081, 82.7862]    [-12.2534, 13.1171, 82.3844]    [-8.74286, 15.1424, 81.8959]    [-5.3173, 17.4283, 82.5775]    [-14.2835, 16.5094, 81.6199]    [-10.6987, 18.393, 80.5823]    [-7.4003, 20.7721, 81.7922]
% [-9.95023, 16.5367, 84.2511]     [-6.43163, 18.6143, 84.1672]    [-2.98908, 20.7024, 83.8703]    [-11.7514, 19.6028, 82.0918]    [-8.46323, 21.8665, 82.9622]    [-4.99998, 24.041, 83.0266]    [-13.901, 23.16, 82.3397]    [-10.5195, 25.3453, 82.7151]    [-6.95768, 27.0157, 81.4008]
% [-9.74316, 16.1843, 82.4465]     [-6.432, 18.4825, 83.7292]    [-2.93692, 20.5305, 82.9717]    [-11.9825, 19.9804, 83.6814]    [-8.55839, 22.2121, 84.1877]    [-4.97019, 23.8384, 82.215]    [-13.9663, 23.3463, 82.9398]    [-10.377, 25.0898, 81.6967]    [-7.14187, 27.7869, 83.5373]
% [-9.66727, 16.1, 81.8414]     [-6.45062, 18.5697, 84.0427]    [-2.97845, 20.8171, 84.2356]    [-12.0584, 20.022, 83.9261]    [-8.47296, 21.9133, 82.9511]    [-5.01175, 24.0219, 82.9565]    [-14.1302, 23.5194, 83.6688]    [-10.4823, 25.2947, 82.5384]    [-7.16131, 27.6641, 83.2106]
% [-9.23158, 22.4586, 85.5783]     [-5.74574, 24.3729, 85.1285]    [-2.30403, 26.4838, 85.0116]    [-11.3428, 26.2185, 86.1987]    [-7.58557, 27.1686, 82.8078]    [-4.31446, 30.0162, 84.9891]    [-13.1507, 29.1449, 84.3525]    [-9.78343, 31.2642, 84.4948]    [-6.35832, 33.3218, 84.329]
% [-9.42291, 23.1145, 88.1343]     [-5.70358, 24.2775, 84.615]    [-2.28219, 26.2782, 84.3628]    [-11.4235, 26.4421, 87.0028]    [-7.95947, 28.4471, 86.6216]    [-4.37921, 30.1513, 85.4619]    [-13.4128, 29.6983, 85.9035]    [-9.79659, 31.429, 84.7238]    [-6.39881, 33.4711, 84.7465]
% [-9.24882, 22.5427, 85.7988]     [-5.71957, 24.545, 85.4805]    [-2.31004, 26.538, 85.2587]    [-11.2107, 25.9831, 85.3749]    [-7.77106, 27.9983, 85.3057]    [-4.28809, 29.7404, 84.192]    [-13.4209, 29.684, 85.891]    [-9.68684, 30.9282, 83.5847]    [-6.44754, 33.5088, 84.8782]
% [-10.9513, -26.1161, 80.0807]     [-7.34012, -23.787, 80.2123]    [-3.87284, -22.0016, 81.9164]    [-13.213, -22.7511, 81.5859]    [-9.40598, -20.1602, 80.5026]    [-5.93396, -18.1874, 81.4978]    [-15.1922, -19.0933, 81.5019]    [-11.4848, -16.5275, 80.5384]    [-7.9646, -14.5155, 81.2132]
% [-10.9513, -26.1161, 80.0807]     [-7.34012, -23.787, 80.2123]    [-3.87284, -22.0016, 81.9164]    [-13.213, -22.7511, 81.5859]    [-9.40598, -20.1602, 80.5026]    [-5.93396, -18.1874, 81.4978]    [-15.1922, -19.0933, 81.5019]    [-11.4848, -16.5275, 80.5384]    [-7.9646, -14.5155, 81.2132]
% [-10.9443, -26.2049, 80.3416]     [-7.33721, -23.8012, 80.0645]    [-3.872, -22.0969, 82.2551]    [-12.9508, -22.2609, 79.8644]    [-9.56167, -20.4867, 81.7007]    [-5.88841, -18.0053, 80.8004]    [-15.2331, -19.0903, 81.7092]    [-11.6814, -16.915, 82.2792]    [-8.10979, -14.694, 82.46]
% [-21.2709, 1.75126, 88.1821]     [-17.733, 3.82241, 87.3992]    [-14.431, 5.95335, 87.7922]    [-23.1233, 5.13818, 87.2273]    [-19.8536, 7.24788, 87.4854]    [-16.3889, 9.33299, 86.9421]    [-25.2393, 8.55369, 87.2696]    [-21.8657, 10.6239, 87.0403]    [-18.5116, 12.757, 87.0154]
% [-21.1384, 1.74721, 87.643]     [-17.789, 3.81917, 87.5997]    [-14.3913, 5.93089, 87.4997]    [-23.2016, 5.1674, 87.4979]    [-19.8766, 7.27641, 87.4919]    [-16.4414, 9.34312, 87.1519]    [-25.3578, 8.61224, 87.6716]    [-21.9399, 10.6906, 87.3693]    [-18.4906, 12.7573, 86.9192]
% [-21.3104, 1.77318, 88.3564]     [-17.791, 3.82927, 87.6187]    [-14.3546, 5.90778, 87.2227]    [-23.2639, 5.18333, 87.7146]    [-19.8748, 7.26978, 87.508]    [-16.4991, 9.37398, 87.3989]    [-25.2187, 8.54887, 87.1918]    [-21.7808, 10.5911, 86.6639]    [-18.4991, 12.7408, 86.9039]
% [-1.85817, 1.6253, 89.8743]     [1.44069, 3.70296, 88.8926]    [4.76553, 5.79232, 88.0016]    [-3.89752, 5.01639, 89.7517]    [-0.601869, 7.13893, 89.2944]    [2.68819, 9.19926, 88.0542]    [-5.9464, 8.47203, 90.2217]    [-2.65436, 10.4745, 88.8913]    [0.65337, 12.6678, 88.5238]
% [-1.87276, 1.63759, 90.2596]     [1.43352, 3.68696, 88.8489]    [4.74815, 5.78729, 87.955]    [-3.90666, 5.02835, 89.6312]    [-0.636901, 7.15401, 89.5535]    [2.70029, 9.25624, 88.6391]    [-5.94846, 8.46745, 90.2219]    [-2.64935, 10.4888, 88.9238]    [0.640938, 12.6141, 88.2976]
% [-1.86961, 1.62746, 89.9785]     [1.44682, 3.70468, 89.354]    [4.76114, 5.80498, 88.2351]    [-3.90418, 5.04656, 89.9141]    [-0.621525, 7.12393, 89.0509]    [2.68747, 9.18518, 87.9124]    [-5.92061, 8.42536, 89.7296]    [-2.64734, 10.4754, 88.7392]    [0.655384, 12.6365, 88.403]
% 
% ];
Mc=zeros(size(M,1)/3,size(M,2));
Tst2cb=zeros(4,4,size(Mc,1));
Tcb2st=zeros(4,4,size(Mc,1));
% gamma3=-0.163;
% gamma3T=[cos(gamma3), -sin(gamma3),0,0;sin(gamma3), cos(gamma3),0,0;0, 0, 1, 0;0, 0, 0, 1];

figure;hold on;xlabel('X');ylabel('Y');zlabel('Z');axis equal;view([45 20]);axis([ -50 50 -50 50 0 150]);
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
%     plotCoord(Tcb2st(:,:,i));
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

Tfilename = join([getenv('VSARMCALIBPATH'), '\T.log']);
psifilename = join([getenv('VSARMCALIBPATH'), '\psi.log']);
fileConfig = fopen(Tfilename,'w');
filePsi = fopen(psifilename,'w');
for i = 1:size(Mc,1)
% % %     Ttc2end(:,:,i)=Ttc2st(:,:,i)*inv(gamma3T);%*inv(uke3T);
%     plotCoord(Ttc2end(:,:,i));
    
    fprintf(fileConfig,'%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n',Tst2cb(:,:,i));
    fprintf(filePsi,'%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n',Cf(i,:));

end
fclose(fileConfig);
fclose(filePsi);
toc; 