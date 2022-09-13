% -----------------------------------------------------------------------------------------------------------------------
% Author : Eunju Jeong (eunju0316@sookmyung.ac.kr)
% 
% input data  : 1) Crazyflie_6DoF_pose.txt 
%                  (unixTimeStamp, r11, r12, r13, tx[m], r21, r22, r23, ty[m], r31, r32, r33, tz[m])
%               2) globalframe_pointcloud.txt
%                  (unixTimeStamp up_x,y,z[m] down_x,y,z[m] left_x,y,z[m] right_x,y,z[m] front_x,y,z[m] back_x,y,z[m])
%               
% output figure : Crazyflie 6DoF motion estimation 
%                 & 6 directions point cloud 
%                 & Wall clustering(Hierarchical clustering) and Line fitting
% -----------------------------------------------------------------------------------------------------------------------
%%
clc;
close all;
clear variables; %clear classes;
rand('state',0); % rand('state',sum(100*clock));
dbstop if error;

%% common setting to read text files

delimiter = ' ';
headerlinesIn = 1;
milliSecondToSecond = 1000;

%% * Parse Crazyflie 6 DoF pose data by Optitrack

% parsing Crazyflie(CF) pose data text file 
textFileDir_optitrack = 'input\FlowdeckTime_Optitrack_Crazyflie_6DoF_pose.txt'

textPoseData_optitrack = importdata(textFileDir_optitrack, delimiter, headerlinesIn);
CFPoseTime_optitrack = textPoseData_optitrack.data(:,1).';
CFPoseTime_optitrack = (CFPoseTime_optitrack - CFPoseTime_optitrack(1)) ./ milliSecondToSecond;
CFPoseData_optitrack = textPoseData_optitrack.data(:,[2:13]);

% Crazyflie pose with various 6-DoF pose representations
numPose_optitrack = size(CFPoseData_optitrack,1);
T_gc_CF_optitrack = cell(1,numPose_optitrack); % initialize
stateEsti_CF_optitrack = zeros(6,numPose_optitrack); % initialize
R_gc_CF_optitrack = zeros(3,3,numPose_optitrack);
for k = 1:numPose_optitrack
    
    % rigid body transformation matrix (4x4)
    T_gc_CF_optitrack{k} = [reshape(CFPoseData_optitrack(k,:).', 4, 3).'; [0, 0, 0, 1]];
    
    % state vector and rotation matrix
    R_gc_CF_optitrack(:,:,k) = T_gc_CF_optitrack{k}(1:3,1:3);
    stateEsti_CF_optitrack(1:3,k) = T_gc_CF_optitrack{k}(1:3,4);
    [yaw_optitrack, pitch_optitrack, roll_optitrack] = dcm2angle(R_gc_CF_optitrack(:,:,k));
    stateEsti_CF_optitrack(4:6,k) = [roll_optitrack; pitch_optitrack; yaw_optitrack];
end

%% * Parse Crazyflie point cloud data (1x19): Optitrack 

% parsing Crazyflie point cloud data text file
textFileDir_pointcloud_Optitrack = 'input\global_pointcloud_1x19_optitrack.txt'

textCFPointCloudData_Optitrack = importdata(textFileDir_pointcloud_Optitrack, delimiter, headerlinesIn);

% Crazyflie 3D point cloud
CFPointCloudData_Optitrack = textCFPointCloudData_Optitrack.data(:,[2:19]); % except for timestamp
numPointCloud_Optitrack = size(CFPointCloudData_Optitrack, 1);
pointcloud_CF_Optitrack = zeros(18, numPointCloud_Optitrack); % initialize

for k = 1:numPointCloud_Optitrack
    pointcloud_CF_Optitrack(:,k)=CFPointCloudData_Optitrack(k,:);
end

%% * Sparse_MW_Mapping parameter

N_GROUP = 8; % the number of cluster for HC
MAX_DISTANCE_RANSAC = 0.01; % max allowable distance for inliers (positive scalar)
NOISE_DISTANCE_TH = 0.4; % the threshold of euclidean distance between the two points
fitted_point_accumulate = [];
num_walls = 1; % initialization (the number of line in 'walls')

%% * Plot trajectory, point cloud, line (moving)

% Define Manhattan frame (MF)
MF_X = [1;0.0012;0.000824]; MF_Y = [-0.0012;1;-0.0015]; MF_Z = [-0.000826;0.0015;1];
MF = [MF_X MF_Y MF_Z];

% 1) play 3D moving trajectory of Crazyflie pose (Figure 10)
figure(1);
figure(2);

%for k = 1:numPose_optitrack
for k = 1: numPose_optitrack %52일 때 line이 처음 생성됨
    figure(1); cla;

    %% draw moving trajectory (optitrack)
    p_gc_CF_optitrack = stateEsti_CF_optitrack(1:3,1:k);
    plot3(p_gc_CF_optitrack(1,:), p_gc_CF_optitrack(2,:), p_gc_CF_optitrack(3,:), 'c', 'LineWidth', 2); hold on; grid on; axis equal;

    %% draw 6 direction point cloud: Optitrack
    sixpoint_CF_Optitrack = pointcloud_CF_Optitrack(:,1:k);
    %plot3(sixpoint_CF_Optitrack(1,:), sixpoint_CF_Optitrack(2,:), sixpoint_CF_Optitrack(3,:), 'b.'); hold on; % up point (+z)
    %plot3(sixpoint_CF_Optitrack(4,:), sixpoint_CF_Optitrack(5,:), sixpoint_CF_Optitrack(6,:), 'b.'); hold on; % down point (-z)
    plot3(sixpoint_CF_Optitrack(7,:), sixpoint_CF_Optitrack(8,:), sixpoint_CF_Optitrack(9,:), 'b.'); hold on; % left point (+y)
    plot3(sixpoint_CF_Optitrack(10,:), sixpoint_CF_Optitrack(11,:), sixpoint_CF_Optitrack(12,:), 'b.'); hold on; % right point (-y)
    plot3(sixpoint_CF_Optitrack(13,:), sixpoint_CF_Optitrack(14,:), sixpoint_CF_Optitrack(15,:), 'b.'); hold on; % front point (+x)
    plot3(sixpoint_CF_Optitrack(16,:), sixpoint_CF_Optitrack(17,:), sixpoint_CF_Optitrack(18,:), 'b.'); hold on; % back point (-x)
    axis equal; hold on;
  
    %% draw camera body and frame (optitrack)
    plot_inertial_frame(0.5); view(47, 48);
    Rgc_CF_current = T_gc_CF_optitrack{k}(1:3,1:3);
    pgc_CF_current = T_gc_CF_optitrack{k}(1:3,4);
    plot_CF_frame(Rgc_CF_current, pgc_CF_current, 0.5, 'm'); hold on;

    %% MW_mapping (Application of ICCAS 2022)
   
    %% 1) Find and eliminate noisy points
    % 결과: points_without_noise

    % (timestamp 제외하고) 1x18 --> 1x2 로 변형
    points = [CFPointCloudData_Optitrack(1:k,7:8); CFPointCloudData_Optitrack(1:k,10:11); CFPointCloudData_Optitrack(1:k,13:14); CFPointCloudData_Optitrack(1:k,16:17)];
    %--------------
    % x,y (left)
    % x,y (right)
    % x,y (front)
    % x,y (back) 
    % 사실 이제 points 에서는 어느 방향인지 상관없음
    %--------------
    numPoints = size(points,1); % the number of 2D points(x,y)

    noise_index = []; % to append the index of noise point
    pair = []; % the nearest two points pair
    %NOISE_DISTANCE_TH = 0.05; % the threshold of euclidean distance between the two points *************
    
    for i = 1:numPoints 
        % 1) i 번째 점을 제외한 점 집합 points_except_i 만들기   
        for j = 1:numPoints
            points_except_i(j,:) = points(j,:);
        end
        points_except_i(i,:) = []; % i번째 행 삭제
        
        % 2) i 번째 점과 가장 가까운 점 찾기
        % return) 'dist' (euclidean distance between point_i and the nearest point), 'nPointIndex_in_points'
        point_i = [points(i,1),points(i,2)]; % i 번째 점 (x,y) 정의
        [nPointIndex_in_points_except_i,dist] = dsearchn(points_except_i, point_i); 
        nPointIndex_in_points = find(points == points_except_i(nPointIndex_in_points_except_i,1)); % points 행렬에서의 index로 변환
        
        pair = [pair;i nPointIndex_in_points]; % 결과) 가장 가까운 점 index들 pair
    
        % 3) 가장 가까운 점과의 거리가 NOISE_DISTANCE_TH를 초과하면 noise로 간주
        % 즉, NOISE_DISTANCE_TH가 작을수록 더 많은 양의 point가 제거된다.
        if dist <= NOISE_DISTANCE_TH
            disp("");
        elseif dist > NOISE_DISTANCE_TH 
            % noise에 i 추가
            noise_index = [noise_index i];
        end
    
    end
    
    numNoise = size(noise_index,2);
    
    points_without_noise = []; % 결과적으로 얻고 싶은 것이 points_without_noise (noise를 제거한 point cloud)
    
    for i = 1:numPoints    
        [query_i] = find(noise_index == i);
        if (query_i) in_noise = 1; % if index i is "in" noise_index, in_noise is 1
        else in_noise = 0; % if index i is "not in" noise_index, in_noise is 0
        end
        
        if in_noise == 1
            continue
        elseif in_noise == 0
            points_without_noise = [points_without_noise; points(i,:)]; % x,y
        end
    end
    numWithoutNoisePoints = size(points_without_noise,1); 

    %% 2) Point clustering and Line fitting

    %% 2-1) Hierarchical clustering (HC)
    Z = linkage(points_without_noise,'ward'); % Ward's method
    %dendrogram(Z) % 여기서 이걸 하면 이 figure가 k개만큼 생성된다.. for문 밖으로 빠져나왔을 때 넣기 
    T = cluster(Z,'Maxclust',N_GROUP); % 최대 N_GROUP개의 그룹 % 행(가로축): index of points, 값: cluster group number
    gscatter(points_without_noise(:,1),points_without_noise(:,2),T) % 그룹별로 색이 지정된 마커를 사용하여 산점도 Plot 그리기
    set(gcf,'Color','w')
    xlabel('X[m]','FontSize',17,'fontname','times new roman')
    ylabel('Y[m]','FontSize',17,'fontname','times new roman')
    zlabel('Z[m]','FontSize',15,'fontname','times new roman')
    set(gca,'FontSize',17,'fontname','times new roman')
    axis equal
    box off
    hold on;

    cluster_all_index = zeros(numWithoutNoisePoints,N_GROUP); %initialization

    for i = 1:numWithoutNoisePoints
        cluster_all_index(i,T(i)) = i; % 행(가로축): index of points, 열(세로축): cluster group number
    end

    % cluster 1번 그룹부터 N_GORUP번 그룹 끝까지 points index가 1열로 나열됨
    cluster_all_index_1Darray = nonzeros(cluster_all_index);
    
    for i = 1:N_GROUP
        cluster_index_sparse = cluster_all_index(:,i); % 의미 없는 0이 많음
        cluster_index = nonzeros(cluster_index_sparse); % 0이 아닌 값만 저장, the index of point per group, 이걸 얻기 위함이었음.
    
        num_cluster_index = size(cluster_index,1);
        points_cluster = zeros(num_cluster_index,2); % initialization
        
        for j = 1:num_cluster_index
            points_cluster(j,:) = [points_without_noise(cluster_index(j),1) points_without_noise(cluster_index(j),2)]; % convert index number to x,y 2D points
        end
    
        % struct 구조로 cluster 정보 저장
        clusterPointStruct(i).groupNumber = i; % cluster group number
        clusterPointStruct(i).pointsIndex = cluster_index; % 그 그룹에 들어있는 점들의 인덱스 (points_without_noise 에서의 인덱스)
        clusterPointStruct(i).numPointsInGroup = num_cluster_index; % 그 그룹에 들어있는 점들의 개수
        clusterPointStruct(i).pointsXY = points_cluster; % 그 그룹에 들어있는 점들의 x,y 좌표 (points_without_noise 에서의 인덱스)
       
        %% 2-1-1) line fitting (RANSAC)
        if num_cluster_index < 50 
            walls_k(i).alignment = [];
            walls_k(i).offset = [];
            walls_k(i).score = [];
            walls_k(i).min_max_endpoints = [];
            continue; % for문 처음(for i = 1:N_GROUP)으로 돌아감.
        else 
            % point가 50개 이상 모이면 벽으로 간주함
            % RANSAC - line fitting 

            %fitted_point_accumulate = [fitted_point_accumulate; points_cluster]; %************* 다시 연구해보기.

            save pointcloud_cluster_1x2.mat points_cluster;
            load pointcloud_cluster_1x2.mat
        
            % Fit a line to the points using the MSAC algorithm.
            % Define the sample size, the maximum distance for inliers,
            % the fit funciton, and the distance evaluation function.
            % Call ransac to run the MSAC algorithm.
            sampleSize = 2; % minimum sample size from data that is required by fitFcn, specified as a positive scalar integer.
           
            fitLineFcn = @(points_cluster) polyfit(points_cluster(:,1),points_cluster(:,2),1); % fit function using polyfit
            
            % distance evaluation function
            evalLineFcn = @(model, points_cluster) sum((points_cluster(:, 2) - polyval(model, points_cluster(:,1))).^2,2);
        
            % inlierIdx: logical 1D array (num_cluster_index x 1), (1: inlier, 0: outlier)
            [modelRANSAC, inlierIdx] = ransac(points_cluster,fitLineFcn,evalLineFcn,sampleSize,MAX_DISTANCE_RANSAC); % do RANSAC
        
            % Refit a line to the inlier using ployfit.
            modelInliers = polyfit(points_cluster(inlierIdx,1),points_cluster(inlierIdx,2),1);
        
            % Display the final fit line. This is robust to the outliers that ransac
            % identified and ignored.        
            inlierPts = points_cluster(inlierIdx,:); % inlier points in points_cluster

            % middle point x, y value
            middle_x = (min(inlierPts(:,1))+ max(inlierPts(:,1)))/2;
            middle_y = modelInliers(1)*middle_x + modelInliers(2);
        
            % struct 구조로 line 정보 저장
            clusterRANSACLineStruct(i).groupNumber = i; % cluster group number
            clusterRANSACLineStruct(i).inlierPoints = inlierPts; % 해당 그룹에서 inlier에 해당하는 점들의 x,y 좌표
            clusterRANSACLineStruct(i).modelInliers = modelInliers; % RANSAC으로 그린 line의 기울기(modelInliers(1)) 및 y 절편(modelInliers(2))
            clusterRANSACLineStruct(i).middlePoint = [middle_x middle_y];
            clusterRANSACLineStruct(i).score = num_cluster_index; % 해당 그룹에 속하는 point 개수
           
            %% Refit slopes - Manhattan frame (MF)

            % 해당 그룹에 속하는 point 개수
            walls_k(i).score = num_cluster_index; 
            
            % Manhattan frame의 X축과의 각도 차이가 30도 미만이면 X축과 평행하도록 기울기 재조정
            slope_line = clusterRANSACLineStruct(i).modelInliers(1); % RANSAC line fitting으로 얻은 line의 기울기
            if angleBetweenTwo3DVectors(MF_X, slope_line) < 30 % [deg]
                refittedSlope = MF_X(2)/MF_X(1); % MF X축과 평행관계로 refit
                walls_k(i).alignment = 'y'; % y축과 수직
                % 0으로 나눌 경우의 예외처리는 안 했음.
            else
                refittedSlope = -1/(MF_X(2)/MF_X(1)); % MF Y축과 평행관계로 refit (MF X축과 수직)
                walls_k(i).alignment = 'x'; % x축과 수직
                % 0으로 나눌 경우의 예외처리는 안 했음.
            end  

            % refitted line --> ax + by + c = 0
            % refittedSlope*x - y + {middle_y - (refittedSlope*middle_x)} = 0
            a = refittedSlope;
            b = -1;
            c = clusterRANSACLineStruct(i).middlePoint(2)-(refittedSlope*clusterRANSACLineStruct(i).middlePoint(1));
            offset = c/sqrt((refittedSlope^2)+1); % MF의 해당 축과의 offset(+,-)
            walls_k(i).offset = offset;

            if walls_k(i).alignment == 'y' % y축과 수직 (즉, x축과 평행)
                x = [min(points_cluster(:,1)) max(points_cluster(:,1))];
                y = a*x + c;

                x_min = min(points_cluster(:,1));
                x_max = max(points_cluster(:,1));
                y_min = a*x_min + c;
                y_max = a*x_max + c;

                walls_k(i).min_max_endpoints = [x_min y_min; x_max y_max];

            elseif walls_k(i).alignment == 'x' % x축과 수직 (즉, y축과 평행)
                x = (y - c)/a;
                y = [min(points_cluster(:,2)) max(points_cluster(:,2))];
                % inlierPts 대신 points_cluster으로 하면 noise는 제외하고 관찰된 모든 point는 커버할 수 있음.

                y_min = min(points_cluster(:,2));
                y_max = max(points_cluster(:,2));
                x_min = (y_min - c)/a;
                x_max = (y_max - c)/a;

                walls_k(i).min_max_endpoints = [x_min y_min; x_max y_max];
            end

            
            %% Visualization - plot the refitted line
            plot3(x, y, [0.3, 0.3], 'k-', 'LineWidth',4) % RANSAC (line fitting) % 3D ver
            % plot(x,y,'k-','LineWidth',4) % RANSAC (line fitting) % 2D ver
            hold on;

%             figure(2); % only plot walls
%             plot(x,y,'k-','LineWidth',2)
%             xlabel('X[m]','FontSize',15,'fontname','times new roman') 
%             ylabel('Y[m]','FontSize',15,'fontname','times new roman')
%             set(gcf,'Color','w')
%             set(gca,'FontSize',15,'fontname','times new roman')
%             axis equal
%             hold on; % 이걸 해야 누적돼서 그려짐
            
        end
    end

    % 이제 여기까지 하면 walls_k가 완성된다.

    num_walls_k = num_walls_k_initianlization; % walls_k에 있는 line 개수
    num_walls_k_initianlization = 0;
    
    % To do list
    % step1) walls_k를 walls에 단순 누적 (처음 walls는 아무 것도 없는 struct)
    % step2) walls에서 line alignment --> 같은 alignment 끼리 offset 비교 && 가장 가까운 endpoint끼리의 거리가 일정거리 이하이면 합치기
    % step3) walls에 있는 line들 plot

    % walls 에 아무것도 없으면 합치지 않고 plot하지도 않고 pass
   
    % step1) walls_k의 line들을 walls에 누적
    for i=1:size(walls_k,2)
        if isempty(walls_k(i).alignment) % 비어있다면
            continue
        else
            %walls_accumulate(num_walls_accumulate) = walls_k(i);
            walls_accumulate(num_walls_accumulate).alignment = walls_k(i).alignment;
            walls_accumulate(num_walls_accumulate).score = walls_k(i).score;
            walls_accumulate(num_walls_accumulate).offset = walls_k(i).offset;
            walls_accumulate(num_walls_accumulate).min_max_endpoints = walls_k(i).min_max_endpoints;
            walls_accumulate(num_walls_accumulate).line_index = num_walls_accumulate;

            num_walls_accumulate = num_walls_accumulate+1;
        end
    end

    % step2) walls_accumulate 에 있는 line들 중에서 같은 벽에 해당하는 것끼리 합치기
    % <<합칠 것들 묶기>>
    % 1) walls_accumulate에서 같은 alignment끼리 묶기 (struct 형태로)
    % output: walls_alignment_y, walls_alignment_x
%     i_y = 1; i_x = 1; % walls_y, walls_x 만들기 위함
%     if num_walls_accumulate == 1 % line이 없어서 아직 walls_accumulate가 생성되지 않았다면
%         disp("");
%     else
%         for i=1:size(walls_accumulate,2) % walls_accumulate에 있는 line 개수 만큼
%             if walls_accumulate(i).alignment == 'y'
%                 walls_alignment_y(i_y).alignment = walls_accumulate(i).alignment;
%                 walls_alignment_y(i_y).offset = walls_accumulate(i).offset;
%                 walls_alignment_y(i_y).min_max_endpoints = walls_accumulate(i).min_max_endpoints;
%                 walls_alignment_y(i_y).index = i_y;
%                 i_y = i_y + 1;
%             elseif walls_accumulate(i).alignment == 'x'
%                 walls_alignment_x(i_x).alignment = walls_accumulate(i).alignment;
%                 walls_alignment_x(i_x).offset = walls_accumulate(i).offset;
%                 walls_alignment_x(i_x).min_max_endpoints = walls_accumulate(i).min_max_endpoints;
%                 walls_alignment_x(i_x).index = i_x;
%                 i_x = i_x + 1;
%             end
%         end
%     end

    % 1) 같은 alignment끼리 line_index 묶기 (output: same_alignment_y, same_alignment_x)
    same_alignment_y = []; same_alignment_x = []; % initialization
    if num_walls_accumulate == 1 % line이 없어서 아직 walls_accumulate가 생성되지 않았다면
        disp("");
    else
        for i=1:size(walls_accumulate,2) % walls_accumulate에 있는 line 개수 만큼
            if walls_accumulate(i).alignment == 'y'
                same_alignment_y = [same_alignment_y; walls_accumulate(i).line_index];
            elseif walls_accumulate(i).alignment == 'x'
                same_alignment_x = [same_alignment_x; walls_accumulate(i).line_index];
            end
        end
    end

    % 2) 같은 alignment끼리 2개씩 한 쌍으로 묶음 --> |offset 차이| 비교를 위함
    % for alignment 'y')
    if size(same_alignment_y,1) == 0 || size(same_alignment_y,1) == 1 disp("")
    else parallelLineOffset_y = nchoosek(same_alignment_y, 2); % nCr, 2개씩 묶음
    end
    % for alignment 'x')
    if size(same_alignment_x,1) == 0 || size(same_alignment_x,1) == 1 disp("")
    else parallelLineOffset_x = nchoosek(same_alignment_x, 2); % nCr, 2개씩 묶음
    end   

    % for alignment 'y')
    if size(same_alignment_y,1) == 0 || size(same_alignment_y,1) == 1 disp("")
    else
        eliminate_line_alignment_y = [];
        % 1) 같은 alignment끼리의 |offset 차이| 저장
        for i = 1:size(parallelLineOffset_y,1)
            line1_index = parallelLineOffset_y(i,1); line2_index = parallelLineOffset_y(i,2);
            parallelLineOffset_y(i,3) = abs(walls_accumulate(line1_index).offset - walls_accumulate(line2_index).offset);
            % 2) |offset 차이|가 PARALLEL_OFFSET_TH 미만이면 하나로 합침
            if parallelLineOffset_y(i,3) <= PARALLEL_OFFSET_TH
                 % 한 line이라도 eliminate_line_alignment_y에 있으면
                if ismember(line1_index, eliminate_line_alignment_y) || ismember(line2_index, eliminate_line_alignment_y)
                    continue;
                else
                    % 추가하기!!) 여기에서 min_max_endpoint 조정해야 한다.
                    eliminate_line_alignment_y = [eliminate_line_alignment_y line2_index]; % 제거할 line 추가
                end
            else continue;
            end
        end % parallelLineOffset_y의 3번째 colum에 |두 line의 offset 차이| 저장
    end
    % for alignment 'x')
    if size(same_alignment_x,1) == 0 || size(same_alignment_x,1) == 1 disp("")
    else
        eliminate_line_alignment_x = [];
        % 1) 같은 alignment끼리의 |offset 차이| 저장
        for i = 1:size(parallelLineOffset_x,1)
            line1_index = parallelLineOffset_x(i,1); line2_index = parallelLineOffset_x(i,2);
            parallelLineOffset_x(i,3) = abs(walls_accumulate(line1_index).offset - walls_accumulate(line2_index).offset);
            % 2) |offset 차이|가 PARALLEL_OFFSET_TH 미만이면 하나로 합침
            if parallelLineOffset_x(i,3) <= PARALLEL_OFFSET_TH
                 % 한 line이라도 eliminate_line_alignment_x에 있으면
                if ismember(line1_index, eliminate_line_alignment_x) || ismember(line2_index, eliminate_line_alignment_x)
                    continue;
                else
                    % 추가하기!!) 여기에서 min_max_endpoint 조정해야 한다.
                    eliminate_line_alignment_x = [eliminate_line_alignment_x line2_index]; % 제거할 line 추가
                end
            else continue;
            end
        end % parallelLineOffset_x의 3번째 colum에 |두 line의 offset 차이| 저장
    end


    refresh; pause(0.01); k
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 5) Plot - Eliminating noisy points
figure;
plot(points(:,1),points(:,2),'.'); % 모든 점들 plot
set(gcf,'Color','w')
axis equal
hold on
for i = 1:numNoise
    plot(points(noise_index(i),1),points(noise_index(i),2),'*g') % 모든 noise 점들 plot
    set(gcf,'Color','w')
    axis equal
end
legend('All Points','Noisy Points')
xlabel('X[m]','FontSize',15,'fontname','times new roman') 
ylabel('Y[m]','FontSize',15,'fontname','times new roman')
set(gcf,'Color','w')
set(gca,'FontSize',15,'fontname','times new roman')


%% 6) Plot Results (trajectory and point cloud)

% plot Crazyflie motion estimation results (Figure 2)
figure;

% 이 부분에 최종 walls plot하기 (hold on)처리

% plot pose of Crazyflie (optitrack)
h_Crazyflie_optitrack = plot3(stateEsti_CF_optitrack(1,:),stateEsti_CF_optitrack(2,:),stateEsti_CF_optitrack(3,:),'c','LineWidth',2); hold on; grid on;

%(point cloud plot version2: Optitrack) visualization of pointcloud_1x19 (with timestamp)
% scatter3(sixpoint_CF_Optitrack(1,:), sixpoint_CF_Optitrack(2,:), sixpoint_CF_Optitrack(3,:), 'b.'); hold on; % up point (+z)
% scatter3(sixpoint_CF_Optitrack(4,:), sixpoint_CF_Optitrack(5,:), sixpoint_CF_Optitrack(6,:), 'b.'); hold on; % down point (-z)
scatter3(sixpoint_CF_Optitrack(7,:), sixpoint_CF_Optitrack(8,:), sixpoint_CF_Optitrack(9,:), 'b.'); hold on; % left point (+y)
scatter3(sixpoint_CF_Optitrack(10,:), sixpoint_CF_Optitrack(11,:), sixpoint_CF_Optitrack(12,:), 'b.'); hold on; % right point (-y)
scatter3(sixpoint_CF_Optitrack(13,:), sixpoint_CF_Optitrack(14,:), sixpoint_CF_Optitrack(15,:), 'b.'); hold on; % front point (+x)
scatter3(sixpoint_CF_Optitrack(16,:), sixpoint_CF_Optitrack(17,:), sixpoint_CF_Optitrack(18,:), 'b.'); hold on; % back point (-x)
axis equal;

% plot inertial frame
plot_inertial_frame(0.5); legend('Optitrack','Point cloud: Optitrack', 'wall'); axis equal; view(26, 73);
%xlabel('x [m]','fontsize',10); ylabel('y [m]','fontsize',10); zlabel('z [m]','fontsize',10); 

gscatter(points_without_noise(:,1),points_without_noise(:,2),T) % 그룹별로 색이 지정된 마커를 사용하여 산점도 Plot 그리기
set(gcf,'Color','w')
xlabel('X[m]','FontSize',17,'fontname','times new roman')
ylabel('Y[m]','FontSize',17,'fontname','times new roman')
set(gca,'FontSize',17,'fontname','times new roman')
axis equal; box off; 
hold off;

% figure options
f = FigureRotator(gca());
