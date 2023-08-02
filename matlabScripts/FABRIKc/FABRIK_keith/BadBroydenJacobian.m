function J_cur=BadBroydenJacobian(dp,dx,J_last)

alpha = 1;
J_cur = J_last + alpha*(dp-J_last*dx)/(dx'*dx)*dx';
end