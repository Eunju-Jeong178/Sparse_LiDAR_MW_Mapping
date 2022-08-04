%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Eunju Jeong (eunju0316@sookmyung.ac.kr)
%
% Description : Find the distance between the two straight parallel lines 
% 
% two parallel lines
% 1) ax + by + c1 = 0
% 2) ax + by + c2 = 0
% distance = |c1-c2|/sqrt(a.^2 + b.^2)
%
% Output 
% 1) parallelOffset : the distance between the two straight parallel lines
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function parallelOffset = parallel_lines_offset(a,b,c1,c2)

    parallelOffset = abs(c1-c2)/sqrt(a.^2 + b.^2);

end