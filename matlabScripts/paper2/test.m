SL = [100 0 20 0 0.2   0  0   0   600 0];

psi = [1 50 1 1 1 1]';
[Tend,S] = FKcc_2segs_nobending_keith(psi,SL,MBP.discrete_element);
% PS_2segs_keith(S,SL,Tend);

[psi1,flag]=IKcc_2segs_nobending_keith(Tend,SL);

font_size = 20;
title('Single Rod v.s. Equivalent Single Rod')
xlabel("x (mm)",'FontName', 'Times New Roman', 'FontSize', font_size)
ylabel("y (mm)",'FontName', 'Times New Roman', 'FontSize', font_size)
zlabel("z (mm)",'FontName', 'Times New Roman', 'FontSize', font_size)
