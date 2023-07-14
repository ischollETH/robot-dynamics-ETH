function C_y = ElemRotY(theta_y)
%   Elementary rotation matrix around y
C_y=[[cos(theta_y),0,sin(theta_y)];[0,1,0];[-sin(theta_y),0,cos(theta_y)]];
end