Tst2tc=[-0.51206    0.000235001    -0.85895    0    
    -0.016658    0.999809    0.010204    0    
    0.858788    0.019534    -0.511958    0    
    -27.0045    8.72514    334.439    1     ]';
num=13;
lg=[19.0242   19.1582 18.8975];
T0 = zeros(4,4);
T0(3,4) = -15+lg(1);
figure; hold on;
axis equal; grid on;
[vecs,Tlm] = returnVecs(num);
T2_13 = Tst2tc\fromVec72Tran(vecs(1,:))*Tlm-T0;
T3_13 = Tst2tc\fromVec72Tran(vecs(2,:))*Tlm-T0;
T4_13 = Tst2tc\fromVec72Tran(vecs(3,:))*Tlm-T0;
T5_13 = Tst2tc\fromVec72Tran(vecs(4,:))*Tlm-T0;
plotCoord_keith(T2_13,1,1);plotCoord_keith(T3_13,1,1);
plotCoord_keith(T4_13,1,1);plotCoord_keith(T5_13,1,1);

num=16;
T0(3,4) = -15+lg(2);
[vecs,Tlm] = returnVecs(num);
T2_16 = Tst2tc\fromVec72Tran(vecs(1,:))*Tlm-T0;
T3_16 = Tst2tc\fromVec72Tran(vecs(2,:))*Tlm-T0;
T4_16 = Tst2tc\fromVec72Tran(vecs(3,:))*Tlm-T0;
T5_16 = Tst2tc\fromVec72Tran(vecs(4,:))*Tlm-T0;
plotCoord_keith(T2_16,2,1);plotCoord_keith(T3_16,2,1);
plotCoord_keith(T4_16,2,1);plotCoord_keith(T5_16,2,1);

num=19;
T0(3,4) = -15+lg(3);
[vecs,Tlm] = returnVecs(num);
T2_19 = Tst2tc\fromVec72Tran(vecs(1,:))*Tlm-T0;
T3_19 = Tst2tc\fromVec72Tran(vecs(2,:))*Tlm-T0;
T4_19 = Tst2tc\fromVec72Tran(vecs(3,:))*Tlm-T0;
T5_19 = Tst2tc\fromVec72Tran(vecs(4,:))*Tlm-T0;
plotCoord_keith(T2_19,3,1);plotCoord_keith(T3_19,3,1);
plotCoord_keith(T4_19,3,1);plotCoord_keith(T5_19,3,1);


plotCoord_keith(eye(4),5,1);

function T=fromVec72Tran(vec)


PP=[34	34	136.5
    -34	34	136.5
    -34	-34	68.5
    34	-34	68.5
    ];
plot3(PP(:,1),PP(:,2),PP(:,3),'k*');

quat=vec(4:7);
R=quat2rotm(quat);
p = vec(1:3)';
T = [R p;[0 0 0 1]];
end



