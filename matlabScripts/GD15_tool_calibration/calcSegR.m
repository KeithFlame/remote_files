function [R]=calcSegR(theta,delta)
R = Expm([0 0 delta]')*Expm([0 theta 0]')*Expm([0 0 -delta]');
end