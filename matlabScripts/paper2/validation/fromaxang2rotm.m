function R = fromaxang2rotm(axang)
%FROMAXANG2ROTM 此处显示有关此函数的摘要
%   此处显示详细说明
ang=norm(axang);
axis=axang/ang;
ang = ang*pi/180;
R = axang2rotm([axis ang]);

end

