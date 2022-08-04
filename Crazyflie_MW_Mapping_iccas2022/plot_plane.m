%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Eunju Jeong (eunju0316@sookmyung.ac.kr)
%
%                 ____________________
%  4 (x4, y4, z4) |                   | 3 (x3, y3, z3)
%                 |                   |
%                 |                   |
%  1 (x1, y1, z1) |___________________| 2 (x2, y2, z2)
%   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% wall color reference
% http://www.n2n.pe.kr/lev-1/color.htm

function plot_plane(x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4)

    vertex = [x1 y1 z1; x2 y2 z2; x3 y3 z3; x4 y4 z4];
    face = [1 2 3 4];
    patch('Faces',face,'Vertices',vertex, 'Facecolor',[1 1 0.95]); % Facecolor: [R G B]
    view(3) % 3D visualization
    hold on

end