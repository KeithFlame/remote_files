function cur_end = P2_backward(last_begin,cur_begin,next_begin,length,limit,flag)
% P1_FORWARD: the expression of the backward reaching phase in the two phases 
% of FABRIK algorithm
% This function contains two (three) different joints
% Input


if(flag == 0) % universal or sphere
    direction_end = next_begin-cur_begin;
    direction_end = direction_end/norm(direction);

    direction_begin = last_begin - cur_begin;
    direction_begin = direction_begin / norm(direction_begin);
    angle = acosd(dot(direction_begin,direction_end));
    if(angle>limit)
        angle_2 = (pi-angle)/2;
        angle_3 = pi-limit-angle_2;
        length_2 = sin(limit)/sin(angle_3);
        e = direction_end-direction_begin;
        direction_end = length_2*e+direction_begin;
        direction_end = direction_end / norm(direction_end);
    end
    cur_end = cur_begin + direction_end*length;
    % cur_end = cur_begin + diretcion*length;


end

end

