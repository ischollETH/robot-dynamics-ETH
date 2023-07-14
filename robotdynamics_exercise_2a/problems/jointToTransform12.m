function T12 = jointToTransform12(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 2 to frame 1. T_12
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  T12 = [[ElemRotY(q(2)),[0;0;0.145]]; [0,0,0,1]];
end