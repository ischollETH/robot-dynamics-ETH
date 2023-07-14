function q_AC = quatMult(q_AB,q_BC)
  % Input: two quaternions to be multiplied
  % Output: output of the multiplication
  
  q = q_AB;
  p = q_BC;
  
  q_0 = q(1); q_n = q(2:4);
  

  q_AC = [[q_0, -q_n']; [q_n, q_0*eye(3)+skewMatrix(q_n)]]*p;
end

