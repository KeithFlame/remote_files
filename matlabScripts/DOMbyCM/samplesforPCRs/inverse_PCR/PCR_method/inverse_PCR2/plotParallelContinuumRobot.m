function plotParallelContinuumRobot(s1,s2)
% this is a function to plot the robot.
%
% input1: s1 the first sub-chain para.
% input2: s2 the second sub-chain para.

m = size(s1,1); 
n = size(s2,1);

hold on;
t = 1;
plot3(s1(t:s1(t,4),1),s1(t:s1(t,4),2),s1(t:s1(t,4),3),'r-');
t = s1(t,4)+1;
plot3(s1(t:s1(t,4),1),s1(t:s1(t,4),2),s1(t:s1(t,4),3),'g-');
t = s1(t,4)+1;
plot3(s1(t:s1(t,4),1),s1(t:s1(t,4),2),s1(t:s1(t,4),3),'b-');
t = s1(t,4)+1;
if(t<m)
    plot3(s1(t:s1(t,4)-1,1),s1(t:s1(t,4)-1,2),s1(t:s1(t,4)-1,3),'c-');
end

t = 1;
plot3(s2(t:s2(t,4),1),s2(t:s2(t,4),2),s2(t:s2(t,4),3),'r-');
t = s2(t,4)+1;
plot3(s2(t:s2(t,4),1),s2(t:s2(t,4),2),s2(t:s2(t,4),3),'g-');
t = s2(t,4)+1;
plot3(s2(t:s2(t,4),1),s2(t:s2(t,4),2),s2(t:s2(t,4),3),'b-');
t = s2(t,4)+1;
if(t<n)
    plot3(s2(t:s2(t,4),1),s2(t:s2(t,4),2),s2(t:s2(t,4),3),'c-');
end



end
