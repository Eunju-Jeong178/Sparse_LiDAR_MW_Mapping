%--------------------------------------------------------------------------
% Description : 
% 
% We consider two lines as the same wall landmark if the
% offset difference between them is less than the threshold and they
% have the same alignment. We then combine the newly detected
% line with the corresponding line in the global MW map (MW_Map_FPLiDAR).
%--------------------------------------------------------------------------

% assign current parameters
xNormalIndex = [];
yNormalIndex = [];
nwalls = length(MW_Map_FPLiDAR);


% find corresponding index
for k = 1:nwalls 
    if (MW_Map_FPLiDAR(k).alignment == 'x')
        xNormalIndex = [xNormalIndex; k];
    elseif (MW_Map_FPLiDAR(k).alignment == 'y')
        yNormalIndex = [yNormalIndex; k];
    end
end


% 1) walls orthogonal to X-axis of MF
if (length(xNormalIndex) == 1) % If the there is only one wall that orthogonal to X-axis, just add it to MW_Map_FPLiDAR

elseif (length(xNormalIndex) > 1)  
    parallelLineOffset_x = nchoosek(xNormalIndex, 2);
    eliminate_line_alignment_x = []; alive_line_alignment_x = []; num_eliminate_line_alignment_x = 0;
    
    % 1) save the |offset difference between same alignments|
    for i = 1:size(parallelLineOffset_x,1)
        line1_index = parallelLineOffset_x(i,1); line2_index = parallelLineOffset_x(i,2);
        parallelLineOffset_x(i,3) = abs(MW_Map_FPLiDAR(line1_index).offset - MW_Map_FPLiDAR(line2_index).offset);
                
        % 2) If |offset difference| is less than parallel_offset_th, combine into one
        if parallelLineOffset_x(i,3) <= parallel_offset_th % remain only the first line
            
            if ismember(line1_index, eliminate_line_alignment_x) || ismember(line2_index, eliminate_line_alignment_x)
                continue;
            else
                eliminate_line_alignment_x = [eliminate_line_alignment_x line2_index]; % line to remove
                num_eliminate_line_alignment_x = num_eliminate_line_alignment_x + 1;
                alive_line_alignment_x = [alive_line_alignment_x line1_index]; alive_line_alignment_x = unique(alive_line_alignment_x); % add line to remain
                    
            end
        else % 둘 다 남김

        end
    end
    MW_Map_FPLiDAR(eliminate_line_alignment_x) = [];
end


% 2) walls orthogonal to Y-axis of MF
if (length(yNormalIndex) == 1) % If the there is only one wall that orthogonal to Y-axis, just add it to MW_Map_FPLiDAR

elseif (length(yNormalIndex) > 1)  
    parallelLineOffset_y = nchoosek(yNormalIndex, 2);
    eliminate_line_alignment_y = []; alive_line_alignment_y = []; num_eliminate_line_alignment_y = 0;

    % 1) save the |offset difference between same alignments|
    for i = 1:size(parallelLineOffset_y,1)
        line1_index = parallelLineOffset_y(i,1); line2_index = parallelLineOffset_y(i,2);
        parallelLineOffset_y(i,3) = abs(MW_Map_FPLiDAR(line1_index).offset - MW_Map_FPLiDAR(line2_index).offset);
                
        % 2) If |offset difference| is less than parallel_offset_th, combine into one
        if parallelLineOffset_y(i,3) <= parallel_offset_th  % remain only the first line
            
            if ismember(line1_index, eliminate_line_alignment_y) || ismember(line2_index, eliminate_line_alignment_y)
                continue;
            else
                eliminate_line_alignment_y = [eliminate_line_alignment_y line2_index]; % line to remove
                num_eliminate_line_alignment_y = num_eliminate_line_alignment_y + 1;
                alive_line_alignment_y = [alive_line_alignment_y line1_index]; alive_line_alignment_y = unique(alive_line_alignment_y); % add line to remain
                    
            end
        else 

        end
    end
    MW_Map_FPLiDAR(eliminate_line_alignment_y) = [];
end
