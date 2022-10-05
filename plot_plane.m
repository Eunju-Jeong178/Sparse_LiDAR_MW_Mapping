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

%function plot_plane(ceiling_height)
    ceiling_height = 2.5;
    
    for i = 1:length(walls)
        if walls(i).alignment == 'x' 
            x_offset = walls(i).offset;
            max_xy_M = walls(i).max_xy_M;
            min_xy_M = walls(i).min_xy_M;
            
            %line([x_offset x_offset],[min_xy_M(2) max_xy_M(2)],'Color','k','LineWidth',4.0); hold on;
            x1 = (min_xy_M(2) - walls(i).refittedLineModel(3))/walls(i).refittedLineModel(1);
            y1 = min_xy_M(2);
            z1 = 0;

            x2 = (max_xy_M(2) - walls(i).refittedLineModel(3))/walls(i).refittedLineModel(1);
            y2 = max_xy_M(2);
            z2 = 0;

            x3 = x2; y3 = y2; z3 = ceiling_height;

            x4 = x1; y4 = y1; z4 = ceiling_height;
            
            vertex = [x1 y1 z1; x2 y2 z2; x3 y3 z3; x4 y4 z4];
            face = [1 2 3 4];
            patch('Faces',face,'Vertices',vertex, 'Facecolor',[0.5 0.5 0.5]); % Facecolor: [R G B]
            view(3); % 3D visualization
            hold on;
    
        elseif walls(i).alignment == 'y'
            y_offset = walls(i).offset;
            max_xy_M = walls(i).max_xy_M;
            min_xy_M = walls(i).min_xy_M;
            
            %line([min_xy_M(1) max_xy_M(1)],[y_offset y_offset],'Color','k','LineWidth',4.0); hold on;
            x1 = min_xy_M(1);
            y1 = walls(i).refittedLineModel(1)*x1 + walls(i).refittedLineModel(3);
            z1 = 0;

            x2 = max_xy_M(1);
            y2 = walls(i).refittedLineModel(1)*x1 + walls(i).refittedLineModel(3);
            z2 = 0;

            x3 = x2; y3 = y2; z3 = ceiling_height;

            x4 = x1; y4 = y1; z4 = ceiling_height;
        
            vertex = [x1 y1 z1; x2 y2 z2; x3 y3 z3; x4 y4 z4];
            face = [1 2 3 4];
            patch('Faces',face,'Vertices',vertex, 'Facecolor',[0.5 0.5 0.5]); % Facecolor: [R G B]
            view(3); % 3D visualization
            hold on;
        end
    end

%end