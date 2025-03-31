function cur_end = P1_forward(cur_begin,next_begin,length,flag)
% P1_FORWARD: one expression of the forward reaching phase in the two phases 
% of FABRIK algorithm
% This function contains two (three) different joints
% Input


if(flag == 0) % universal or sphere
    direction = next_begin-cur_begin;
    diretcion = direction/norm(direction);
    cur_end = cur_begin + diretcion*length;
end

end