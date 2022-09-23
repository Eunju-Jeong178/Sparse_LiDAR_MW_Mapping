
for i = 1:length(walls)
    if walls(i).alignment == 'x' 
        x_offset = walls(i).offset;
        max_xy_M = walls(i).max_xy_M;
        min_xy_M = walls(i).min_xy_M;
        
        line([x_offset x_offset],[min_xy_M(2) max_xy_M(2)],'Color','k','LineWidth',4.0); hold on;
    elseif walls(i).alignment == 'y'
        y_offset = walls(i).offset;
        max_xy_M = walls(i).max_xy_M;
        min_xy_M = walls(i).min_xy_M;
        
        line([min_xy_M(1) max_xy_M(1)],[y_offset y_offset],'Color','k','LineWidth',4.0); hold on;

    end
end