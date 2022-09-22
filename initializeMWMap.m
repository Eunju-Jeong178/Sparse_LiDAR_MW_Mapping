function [ManhattanWorldMap] = initializeMWMap(walls,PARALLEL_OFFSET_TH)

% assign current parameters
ManhattanWorldMap = [];
xNormalIndex = [];
yNormalIndex = [];
nwalls = length(walls);


% find corresponding index
for k = 1:nwalls % walls 개수만큼 반복, alignment끼리 index 묶음
    if (walls(k).alignment == 'x')
        xNormalIndex = [xNormalIndex; k];
    elseif (walls(k).alignment == 'y')
        yNormalIndex = [yNormalIndex; k];
    end
end


% x normal walls
if (length(xNormalIndex) == 1) % 그 alignmnent에 하나 밖에 없다면 그냥 ManhattanWorldMap에 합침
    ManhattanWorldMap = [ManhattanWorldMap; walls(xNormalIndex(1))];
elseif (length(xNormalIndex) > 1)  
    parallelLineOffset_x = nchoosek(xNormalIndex, 2);
    
    eliminate_line_alignment_x = []; alive_line_alignment_x = []; num_eliminate_line_alignment_x = 0;
    % 1) 같은 alignment끼리의 |offset 차이| 저장
    for i = 1:size(parallelLineOffset_x,1)
        line1_index = parallelLineOffset_x(i,1); line2_index = parallelLineOffset_x(i,2);
        parallelLineOffset_x(i,3) = abs(walls(line1_index).offset - walls(line2_index).offset);
                
        % 2) |offset 차이|가 PARALLEL_OFFSET_TH 미만이면 하나로 합침
        if parallelLineOffset_x(i,3) <= PARALLEL_OFFSET_TH % 여기에 가장 가까운 끝점 사이의 거리가 얼마 미만이면 합침도 &&으로 추가하기*********
            % 한 line이라도 eliminate_line_alignment_y에 있으면
            if ismember(line1_index, eliminate_line_alignment_x) || ismember(line2_index, eliminate_line_alignment_x)
                continue;
            else
                % line1만 남김.
                ManhattanWorldMap = [ManhattanWorldMap; walls(line1_index)];
                        
                eliminate_line_alignment_x = [eliminate_line_alignment_x line2_index]; % 제거할 line 추가
                num_eliminate_line_alignment_x = num_eliminate_line_alignment_x + 1;
                alive_line_alignment_x = [alive_line_alignment_x line1_index]; alive_line_alignment_x = unique(alive_line_alignment_x); % 유지할 line 추가
                    
            end
        else 
            ManhattanWorldMap = [ManhattanWorldMap; walls(line1_index)]; % |offset 차이|가 PARALLEL_OFFSET_TH 이상이면 다른 벽으로 취급하고 합치지 않음
            ManhattanWorldMap = [ManhattanWorldMap; walls(line2_index)];
        end
    end 
end



% y normal walls
if (length(yNormalIndex) == 1) % 그 alignmnent에 하나 밖에 없다면 그냥 ManhattanWorldMap에 합침
    ManhattanWorldMap = [ManhattanWorldMap; walls(yNormalIndex(1))];
elseif (length(yNormalIndex) > 1)  
    parallelLineOffset_y = nchoosek(yNormalIndex, 2);
    
    eliminate_line_alignment_y = []; alive_line_alignment_y = []; num_eliminate_line_alignment_y = 0;
    % 1) 같은 alignment끼리의 |offset 차이| 저장
    for i = 1:size(parallelLineOffset_y,1)
        line1_index = parallelLineOffset_y(i,1); line2_index = parallelLineOffset_y(i,2);
        parallelLineOffset_y(i,3) = abs(walls(line1_index).offset - walls(line2_index).offset);
                
        % 2) |offset 차이|가 PARALLEL_OFFSET_TH 미만이면 하나로 합침
        if parallelLineOffset_y(i,3) <= PARALLEL_OFFSET_TH % 여기에 가장 가까운 끝점 사이의 거리가 얼마 미만이면 합침도 &&으로 추가하기*********
            % 한 line이라도 eliminate_line_alignment_y에 있으면
            if ismember(line1_index, eliminate_line_alignment_y) || ismember(line2_index, eliminate_line_alignment_y)
                continue;
            else
                % line1만 남김.
                ManhattanWorldMap = [ManhattanWorldMap; walls(line1_index)];
                        
                eliminate_line_alignment_y = [eliminate_line_alignment_y line2_index]; % 제거할 line 추가
                num_eliminate_line_alignment_y = num_eliminate_line_alignment_y + 1;
                alive_line_alignment_y = [alive_line_alignment_y line1_index]; alive_line_alignment_y = unique(alive_line_alignment_y); % 유지할 line 추가
                    
            end
        else 
            ManhattanWorldMap = [ManhattanWorldMap; walls(line1_index)]; % |offset 차이|가 PARALLEL_OFFSET_TH 이상이면 다른 벽으로 취급하고 합치지 않음
            ManhattanWorldMap = [ManhattanWorldMap; walls(line2_index)];
        end
    end 
end


end