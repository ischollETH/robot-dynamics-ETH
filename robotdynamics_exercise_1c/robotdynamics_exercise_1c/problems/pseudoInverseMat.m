function [ pinvA ] = pseudoInverseMat(A, lambda)
% Input: Any m-by-n matrix, and a damping factor.
% Output: An n-by-m pseudo-inverse of the input according to the Moore-Penrose formula

% Get the number of rows (m) and columns (n) of A
[m, n] = size(A);

if (m >= n)
    pinvA = (A'*A + (lambda^2)*eye(n))\A';
else
    pinvA = A'/(A*A' + (lambda^2)*eye(m));
end
end
