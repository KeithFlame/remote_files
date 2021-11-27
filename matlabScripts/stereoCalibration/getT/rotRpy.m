function rot = rotRpy(rpyAngles)

phi = rpyAngles(1);
theta = rpyAngles(2);
psi = rpyAngles(3);

rot = [cos(phi)*cos(theta), -sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi), sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi);
    sin(phi)*cos(theta), cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi), -cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi);
-sin(theta), cos(theta)*sin(psi), cos(theta)*cos(psi)];

end
