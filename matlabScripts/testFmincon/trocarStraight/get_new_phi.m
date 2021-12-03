function phi=get_new_phi(last_phi,current_phi)
if(last_phi>pi)
    last_phi=last_phi-2*pi;
elseif(last_phi<-pi)
    last_phi=last_phi+2*pi;
end
if(current_phi>pi)
    current_phi=current_phi-2*pi;
elseif(current_phi<-pi)
    current_phi=current_phi+2*pi;
end
phi=current_phi-(current_phi-last_phi)*0.01;
end