%--------------------------------------------------------------------------
% Description:
%   draw walls orthogonal to the each axis of the global Manhattan frame(MF)
%--------------------------------------------------------------------------
    
for i = 1:length(MW_Map_FPLiDAR)
    % The wall orthogonal to the X-axis of the MF
    if MW_Map_FPLiDAR(i).alignment == 'x' 
        x_offset = MW_Map_FPLiDAR(i).offset;
        max_xyz_M = MW_Map_FPLiDAR(i).max_xyz_M;
        min_xyz_M = MW_Map_FPLiDAR(i).min_xyz_M;
        
        line([x_offset x_offset],[min_xyz_M(2) max_xyz_M(2)],'Color','k','LineWidth',4.0); hold on;

    % The wall orthogonal to the Y-axis of the MF
    elseif MW_Map_FPLiDAR(i).alignment == 'y'
        y_offset = MW_Map_FPLiDAR(i).offset;
        max_xyz_M = MW_Map_FPLiDAR(i).max_xyz_M;
        min_xyz_M = MW_Map_FPLiDAR(i).min_xyz_M;
        
        line([min_xyz_M(1) max_xyz_M(1)],[y_offset y_offset],'Color','k','LineWidth',4.0); hold on;
    end
end
