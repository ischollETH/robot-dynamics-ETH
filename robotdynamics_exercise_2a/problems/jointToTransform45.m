function T45 = jointToTransform45(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 5 to frame 4. T_45
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  T45 = [[ElemRotY(q(5)),[0.168;0;0]];[0,0,0,1]];
end

