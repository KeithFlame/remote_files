function u = Actuation2Curvature_keith(qa,MBP)
% this is function to convert the curvature of continuum manipulator to the
% actuation length with several variables.
%
% input1: qa the actuation variables (6 X 1 vector) (rad, m)
% input2: MBP the multi-backbone parameters (a structural variable). 
%
% output: u curvature of the 2-seg continuum manipulator (6 X 1 vector) (1/m)
%
%
% Author: Keith W.
% Ver. 1.0
% Date: 15.02.2022

G = MBP.Gc;
u=G\qa(3:end);

end
