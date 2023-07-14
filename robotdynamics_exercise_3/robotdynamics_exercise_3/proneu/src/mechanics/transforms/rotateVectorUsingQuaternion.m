% proNEu: A tool for rigid multi-body mechanics in robotics.
% 
% Copyright (C) 2017  Marco Hutter, Christian Gehring, C. Dario Bellicoso,
% Vassilios Tsounis
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.

function A_r = rotateVectorUsingQuaternion(q_AB, B_r)
% ROTATEVECTORUSINGQUATERNION(q_AB, B_r) projects the components of B_r to
% those of A_r.
%
% Author(s): Dario Bellicoso

q0 = q_AB(1);
qv = q_AB(2:4);
qv = qv(:);
sQv = skew(qv);

B_r = B_r(:);

A_r = (2*q0^2-1)*B_r + 2*q0*sQv*B_r + 2*qv*(qv'*B_r);

end
