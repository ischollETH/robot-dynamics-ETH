function C_x = ElemRotX(theta_x)
%   Elementary rotation matrix around x
C_x=[[1,0,0];[0,cos(theta_x),-sin(theta_x)];[0,sin(theta_x),cos(theta_x)]];
end

