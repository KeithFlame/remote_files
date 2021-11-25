clc;clear;
theta1=1.1/2*pi*0.705;
delta1=-pi/2;
theta2=1.3*pi*0.606;
delta2=3*pi/4;
e1=[1 0 0]';e2=[0 1 0]';e3=[0 0 1]';
MP.E=50e9;%Rod Young's modules
MP.mu=0.33;%Rod Poisson rate
MP.d=0.37e-3;%Rod diameter
MP.rho=0.85e-3;%Rod pitch circle radius
MP.L=500e-3;%Robot stem length
MP.L1=39e-3;%Robot seg1 length
MP.L2=25e-3;%Robot seg2 length
MP.Lr=4e-3;%Robot rigid seg length
MP.Lg=8e-3;%Robot gipper length
MP.r11=[1 0 0]'*MP.rho;MP.r12=[0 1 0]'*MP.rho;
MP.r21=[1/sqrt(2) 1/sqrt(2) 0]'*MP.rho;MP.r22=[-1/sqrt(2) 1/sqrt(2) 0]'*MP.rho;
MP.Q1=[S(MP.r11)*e3 S(MP.r12)*e3 S(MP.r21)*e3 S(MP.r22)*e3];
MP.Q2=[S(MP.r21)*e3 S(MP.r22)*e3];
MP.I=pi*MP.d^4/64;MP.A=pi*MP.d^2/4;MP.G=MP.E/2/(1+MP.mu);MP.J=2*MP.I;
MP.Ke=diag([3e8 3e8 MP.A*MP.E]);
MP.Kb=diag([MP.E*MP.I MP.E*MP.I 10]);
p=zeros(67,3);
Rs=zeros(67,9);Rs(1,:)=[1 0 0 0 1 0 0 0 1];
for i=1:39
    if(theta1~=0)
    p(i+1,:)=MP.L1/theta1*[cos(delta1)*(1-cos(theta1*i/39)) -sin(delta1)*(1-cos(theta1*i/39)) sin(theta1*i/39)];
    R=Expm([0 0 -delta1]')*Expm([0 theta1*i/39 0]')*Expm([0 0 delta1]');
    Rs(i+1,:)=[R(:,1)' R(:,2)' R(:,3)'];
    else
        p(i+1,:)=[0 0 i/39]*MP.L1;
        R=eye(3);
        Rs(i+1,:)=[1 0 0 0 1 0 0 0 1];
    end
end
R1=R;
p(41,:)=p(40,:)+[0 0 MP.Lr]*R1';
Rs(41,:)=Rs(40,:);
p1=p(41,:);
p(42,:)=p(41,:);Rs(42,:)=Rs(41,:);
for i=42:66
    p0=MP.L2/theta2*[cos(delta2)*(1-cos(theta2*(i-41)/25)) -sin(delta2)*(1-cos(theta2*(i-41)/25)) sin(theta2*(i-41)/25)];
    p(i+1,:)=p0*R1'+p1;
    R=Expm([0 0 -delta2]')*Expm([0 theta2*(i-41)/25 0]')*Expm([0 0 delta2]');
    R=R1*R;
    Rs(i+1,:)=[R(:,1)' R(:,2)' R(:,3)'];
end

plotShape(p,Rs,[41 26],MP,[0.93 0.69 0.13]);