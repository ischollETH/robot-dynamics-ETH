function R = quatToRotMat(q)
  % Input: quaternion [w x y z]
  % Output: corresponding rotation matrix
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  q_0 = q(1); q_n = q(2:4);
  R = (2.0*(q_0^2)-1)*eye(3) + 2.0*q_0*skewMatrix(q_n) + 2.0*q_n*(q_n');
end