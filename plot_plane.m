%--------------------------------------------------------------------------
% Description:
%                 ____________________
%  4 (x4, y4, z4) |                   | 3 (x3, y3, z3)
%                 |                   |
%                 |                   |
%  1 (x1, y1, z1) |___________________| 2 (x2, y2, z2)
%   
%--------------------------------------------------------------------------

% wall color reference
% (http://www.n2n.pe.kr/lev-1/color.htm)

ceiling_height = 2.5; 
    
for i = 1:length(MW_Map_FPLiDAR)
    if MW_Map_FPLiDAR(i).alignment == 'x' 
        x_offset = MW_Map_FPLiDAR(i).offset;
        max_xy_M = MW_Map_FPLiDAR(i).max_xyz_M;
        min_xy_M = MW_Map_FPLiDAR(i).min_xyz_M;
            
        x1 = x_offset;
        y1 = min_xy_M(2);
        z1 = 0;

        x2 = x_offset;
        y2 = max_xy_M(2);
        z2 = 0;

        x3 = x2; y3 = y2; z3 = ceiling_height;

        x4 = x1; y4 = y1; z4 = ceiling_height;
            
        vertex = [x1 y1 z1; x2 y2 z2; x3 y3 z3; x4 y4 z4];
        face = [1 2 3 4];
        patch('Faces',face,'Vertices',vertex, 'Facecolor',[1 0 0]); % Facecolor: [R G B] 
        view(3); % 3D visualization
        hold on;
    
    elseif MW_Map_FPLiDAR(i).alignment == 'y'
        y_offset = MW_Map_FPLiDAR(i).offset;
        max_xy_M = MW_Map_FPLiDAR(i).max_xyz_M;
        min_xy_M = MW_Map_FPLiDAR(i).min_xyz_M;
            
        x1 = min_xy_M(1);
        y1 = y_offset;
        z1 = 0;

        x2 = max_xy_M(1);
        y2 = y_offset;
        z2 = 0;

        x3 = x2; y3 = y2; z3 = ceiling_height;

        x4 = x1; y4 = y1; z4 = ceiling_height;
        
        vertex = [x1 y1 z1; x2 y2 z2; x3 y3 z3; x4 y4 z4];
        face = [1 2 3 4];
        patch('Faces',face,'Vertices',vertex, 'Facecolor',[0 1 0]); % Facecolor: [R G B]
        view(3); % 3D visualization
        hold on;
    end
end
