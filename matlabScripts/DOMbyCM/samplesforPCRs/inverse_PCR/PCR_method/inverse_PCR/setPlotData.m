function [s1,s2] = setPlotData(y0, y1,y2,y3)
% this is a function to calculate the line to plot the parallel continuum
% tructure.
%
% input1: y0 the base stem
% input2: y1 the seg1
% input3: y2 the seg2
% input4: y3 the equivalent rod
%
% output1: lines for sub-chain1
% output2: lines for sub-chain2
%
% Author Keith W.
% Ver. 1.0
% Date 03.31.2023


is_y0 = 1;
if(sum(abs(y0(1,1:3))<1e-3))
    is_y0 = 0;
end
m0 = 0;
m0_ = 0;
if(is_y0)
    m0 = find(all(y0==0,2));
    m0_ = size(y0,1)-m0;
    s10 = [y0(1:m0-1,1:3) (m0-1)*ones(m0-1,1)];
    s20 = [y0(m0+1:end,1:3) m0_*ones(m0_,1)];
else
    s10 = [];
    s20 = [];
end
m1 = find(all(y1==0,2));
m1_ = size(y1,1)-m1 + m0_;
s11 = [y1(1:m1-1,1:3) (m1-1 + m0)*ones(m1-1,1)];
s21 = [y1(m1+1:end,1:3) m1_*ones(m1_-m0_,1)];

m2 = find(all(y2==0,2));
m2_ = size(y2,1)-m2+m1_;
s12 = [y2(1:m2-1,1:3) (m2-1 + m1-1 + m0)*ones(m2-1,1)];
s22 = [y2(m2+1:end,1:3) m2_*ones(m2_-m1_,1)];
m3 = find(all(y3==0,2));
m3_ = size(y3,1)-m3 + m2_;
s13 = [y3(1:m3-1,1:3) (m3-1 + m2-1 + m1-1 + m0)*ones(m3-1,1)];
s23 = [y3(m3+1:end,1:3) (m3_)*ones(m3_-m2_,1)];

s1 = [s10;s11;s12;s13];
s2 = [s20;s21;s22;s23];
end