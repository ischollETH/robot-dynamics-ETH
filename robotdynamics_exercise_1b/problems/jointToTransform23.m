function T23 = jointToTransform23(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 3 to frame 2. T_23
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  T23 = [[ElemRotY(q(3)),[0;0;0.270]];[0,0,0,1]];
end
