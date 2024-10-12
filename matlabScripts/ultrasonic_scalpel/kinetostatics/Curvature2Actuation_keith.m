function qa = Curvature2Actuation_keith(u,MBP)
% this is function to convert the curvature of continuum manipulator to the
% actuation length with several variables.
%
% input1: u curvature of the 2-seg continuum manipulator (6 X 1 vector) (1/m)
% input2: MBP the multi-backbone parameters (a class variable)
%
% output: QA the actuation variables (6 X 1 vector) (m)
%
%
% Author: Keith W.
% Ver. 1.0
% Date: 15.02.2022

G = MBP.Gc;
qa=G*u;
end
