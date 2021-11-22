function calcInnerCirculationAllVaribale(ui,Pi,Ri,dsi,ui0,Pi0,Ri0,dsi0)

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.22.2021
% Ver. 1.0
% input1: curvature
% input2: posistion
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
[u,P,R,ds]=getCurrentVariables(ui,Pi,Ri,dsi);
[u0,P0,R0,ds0]=getPreoptimalVariables(ui0,Pi0,Ri0,dsi0);
structure_para=getStructurePara;
getK(structure_para);
getg(u,u0);
getJp(u,P,R,ds);
getXk(R0,P);
getxkT(R0,P0);

end