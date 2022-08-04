%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Eunju Jeong (eunju0316@sookmyung.ac.kr)
% Date : 2022.06.07 - 2022.06.16
% Description : 
% !!Back up!!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; clc;
%% common setting to read text files
delimiter = ','; % txt file에서 comma 제거
headerlinesIn = 1;

%% 1) Parse point cloud data

% with comma txt file
textGlobalPointCloudYFileDir = 'input\0616_B201_06_pyresult_globalpointcloud_1x3.txt'; % final B201 --> structure1
%textGlobalPointCloudYFileDir = 'input\0618_floor2_test02_pyresult_globalpointcloud_1x3.txt'; % final 2F, fig_intro --> structure4

textGlobalPointCloudData = importdata(textGlobalPointCloudYFileDir, delimiter, headerlinesIn);
points = textGlobalPointCloudData.data(:,[1:2]); % x,y
points3D = textGlobalPointCloudData.data(:,[1:3]); % x,y,z
numPoints = size(points,1);

save pointcloud_1x2.mat points; % save 2D point cloud as mat file

rng('default'); % For reproducibility
load pointcloud_1x2.mat

%% parameters

% %final: 2F_L --> 0.5m 늘림
% NOISE_DISTANCE_TH = 0.04; % 2) the threshold of euclidean distance between the two points
% N_GROUP = 10; % 3-2) the number of sub group (cluster)
% MAX_DISTANCE_RANSAC = 0.1; % 4) max allowable distance for inliers (positive scalar)
% ANGLE_TH = 45; % 5) Refit slope: the angle between two lines [deg] 
% PARALLEL_OFFSET_TH = 1.5; % 6-3) 평행한 두 직선 간의 거리가 이것보다 작으면 한 직선으로 합침

% final_B201
NOISE_DISTANCE_TH = 0.27; % 2) the threshold of euclidean distance between the two points
N_GROUP = 4; % 3-2) the number of sub group (cluster)
MAX_DISTANCE_RANSAC = 0.05; % 4) max allowable distance for inliers (positive scalar)
ANGLE_TH = 45; % 5) Refit slope: the angle between two lines [deg] 
PARALLEL_OFFSET_TH = 0.05; % 6-3) 평행한 두 직선 간의 거리가 이것보다 작으면 한 직선으로 합침

ceiling_height = 2.6; %(up distance + down distance) 이건 나중에 파일 읽어와서 자동적으로 정하도록 변수화하자. (2층: 3.2m, 지하2층: 2.6m)

%% 2) Find noise points

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

    % 3) 가장 가까운 점과의 거리가 DISTANCE_TH를 초과하면 noise로 간주
    if dist <= NOISE_DISTANCE_TH
        disp("");
    elseif dist > NOISE_DISTANCE_TH 
        % noise에 i 추가
        noise_index = [noise_index i];
    end

end

% plot all points and noise points
numNoise = size(noise_index,2);

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
legend('All Points','Noise Points')
xlabel('X[m]','FontSize',15,'fontname','times new roman') 
ylabel('Y[m]','FontSize',15,'fontname','times new roman')
set(gcf,'Color','w')
set(gca,'FontSize',15,'fontname','times new roman')

% noise를 제거한 point cloud
points_without_noise = [];

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

% Figure)
figure;
scatter(points(:,1),points(:,2),'.'); % noise points를 제거한 점들
axis equal
title('2D point cloud'); 
xlabel('X[m]','FontSize',17,'fontname','times new roman') 
ylabel('Y[m]','FontSize',17,'fontname','times new roman')
set(gcf,'Color','w')
set(gca,'FontSize',17,'fontname','times new roman')
figure;

% Figure)
plot3(points3D(:,1),points3D(:,2),points3D(:,3),'b.')
xlabel('X[m]','FontSize',17,'fontname','times new roman')
ylabel('Y[m]','FontSize',17,'fontname','times new roman')
zlabel('Z[m]','FontSize',17,'fontname','times new roman')
set(gcf,'Color','w')
set(gca,'FontSize',17,'fontname','times new roman')
grid off
title('3D point cloud'); 
box off;

% Figure 1)
figure;
scatter(points_without_noise(:,1),points_without_noise(:,2),'.'); % noise points를 제거한 점들
axis equal
title('2D point cloud without noise');
xlabel('X[m]','FontSize',17,'fontname','times new roman')  
ylabel('Y[m]','FontSize',17,'fontname','times new roman') 
set(gcf,'Color','w')
set(gca,'FontSize',17,'fontname','times new roman')
box off
figure;

%% 3) 전체 point cloud에서 noise를 제거하고 clustering 시행 
%% 3-1) Hierarchical clustering - 1 linkage)

% 객체를 계층적 이진 군집(euclidean distance가 가장 작은 2개의 점 군집) 트리로 그룹화

% Figure 2)
Z = linkage(points_without_noise,'ward'); % Ward's method
dendrogram(Z)
%dendrogram(Z,'ColorThreshold','default');
set(gcf,'Color','w')
xlabel('Data Points','FontSize',17,'fontname','times new roman')
ylabel('Cluster Distance','FontSize',17,'fontname','times new roman')
set(gca,'FontSize',17,'fontname','times new roman')
figure;
% 가로축: 원래 데이터 세트에 포함된 객체의 인덱스
% 세로축: 객체 간 거리

%% 3-2) Hierarchical clustering - 2 points cluster)

% 계층적 트리를 여러 군집으로 나누기 위한 절단 위치 결정

% Figure 3)
T = cluster(Z,'Maxclust',N_GROUP); % 최대 N_GROUP개의 그룹 % 행(가로축): index of points, 값: cluster group number
gscatter(points_without_noise(:,1),points_without_noise(:,2),T) % 그룹별로 색이 지정된 마커를 사용하여 산점도 Plot 그리기
set(gcf,'Color','w')
xlabel('X[m]','FontSize',17,'fontname','times new roman')
ylabel('Y[m]','FontSize',17,'fontname','times new roman')
set(gca,'FontSize',17,'fontname','times new roman')
axis equal
box off
figure;

% T = cluster(Z,'cutoff',3,'depth',4); 
% gscatter(points(:,1),points(:,2),T) % 그룹별로 색이 지정된 마커를 사용하여 산점도 Plot 그리기
% figure;

cluster_all_index = zeros(numWithoutNoisePoints,N_GROUP); %initialization

for i = 1:numWithoutNoisePoints
    cluster_all_index(i,T(i)) = i; % 행(가로축): index of points, 열(세로축): cluster group number
end

% cluster 1번 그룹부터 N_GORUP번 그룹 끝까지 points index가 1열로 나열됨
cluster_all_index_1Darray = nonzeros(cluster_all_index);

for i = 1:N_GROUP
    cluster_index_sparse = cluster_all_index(:,i); % 의미 없는 0이 많음
    cluster_index = nonzeros(cluster_index_sparse); % 0이 아닌 값만 저장

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

    %% 4) RANSAC - line fitting 
    % Figure 4)
    %plot_ransac(points_cluster) % cluster 된 points 그룹끼리 RANSAC (line fitting) & plane 으로 만들어줌
    
    save pointcloud_cluster_1x2.mat points_cluster;
    load pointcloud_cluster_1x2.mat
    plot(points_cluster(:,1),points_cluster(:,2),'.'); % x, y
    xlabel('X[m]','FontSize',17,'fontname','times new roman')
    ylabel('Y[m]','FontSize',17,'fontname','times new roman')
    set(gcf,'Color','w')
    set(gca,'FontSize',17,'fontname','times new roman')
    axis equal
    hold on
    box off

    % Fit a line to the points using the MSAC algorithm.
    % Define the sample size, the maximum distance for inliers,
    % the fit funciton, and the distance evaluation function.
    % Call ransac to run the MSAC algorithm.
    sampleSize = 2; % minimum sample size from data that is required by fitFcn, specified as a positive scalar integer.
    %MAX_DISTANCE_RANSAC = 0.07; % max allowable distance for inliers (positive scalar) ************************

    fitLineFcn = @(points) polyfit(points(:,1),points(:,2),1); % fit function using polyfit
    
    % distance evaluation function
    evalLineFcn = @(model, points) sum((points(:, 2) - polyval(model, points(:,1))).^2,2);

    % inlierIdx: logical 1D array (num_cluster_index x 1), (1: inlier, 0: outlier)
    [modelRANSAC, inlierIdx] = ransac(points_cluster,fitLineFcn,evalLineFcn,sampleSize,MAX_DISTANCE_RANSAC); % do RANSAC

    % Refit a line to the inlier using ployfit.
    modelInliers = polyfit(points_cluster(inlierIdx,1),points_cluster(inlierIdx,2),1);

    % Display the final fit line. This is robust to the outliers that ransac
    % identified and ignored.

    inlierPts = points_cluster(inlierIdx,:); % inlier points in points_cluster
    x = [min(inlierPts(:,1)) max(inlierPts(:,1))]; % range of x, the size of line
    y = modelInliers(1)*x + modelInliers(2); % y = ax + b, slope: modelInliers(1)

    % middle point x, y value
    middle_x = (min(inlierPts(:,1))+ max(inlierPts(:,1)))/2;
    middle_y = modelInliers(1)*middle_x + modelInliers(2);

    % struct 구조로 line 정보 저장
    clusterLineStruct(i).groupNumber = i; % cluster group number
    clusterLineStruct(i).inlierPoints = inlierPts; % 그 그룹에서 inlier에 해당하는 점들의 x,y 좌표
    clusterLineStruct(i).modelInliers = modelInliers; % RANSAC으로 그린 line의 기울기(modelInliers(1)) 및 y 절편(modelInliers(2))
    clusterLineStruct(i).middlePoint = [middle_x middle_y];
   
    % line visualization
    plot(x, y, 'b-', 'LineWidth',2.5) % RANSAC (line fitting)
    xlabel('X[m]','FontSize',17,'fontname','times new roman')
    ylabel('Y[m]','FontSize',17,'fontname','times new roman')
    set(gcf,'Color','w')
    set(gca,'FontSize',17,'fontname','times new roman')
    axis equal
    %legend('Points','Robust fit');
    hold on
    box off

end

%% 5) Refit slopes - Manhattan World

numPointsInGroup = [];
% cluster 된 그룹 중 가장 point의 개수가 많은 그룹을 n으로 저장
numPointsInGroup = [numPointsInGroup clusterPointStruct.numPointsInGroup];

for i = 1 : N_GROUP
    n = find(numPointsInGroup == max(numPointsInGroup)); % 가장 많은 점을 가지고 있는 group
end

%n = 1;
m1 = clusterLineStruct(n).modelInliers(1); %기준 기울기로 선택
        
for i = 1 : N_GROUP
    m2 = clusterLineStruct(i).modelInliers(1);
    
    % ANGLE_TH를 기준으로 수직 관계, 평행 관계 결정
    if angleBetweenTwoVectors(m1,m2) > ANGLE_TH
        % 수직관계로 refit
        m2 = -1/m1;
    else
        % 평행관계로 refit
        m2 = m1;
    end
    
    % output
    refitLineStruct(i).groupNumber = i;
    refitLineStruct(i).refitSlope = m2; % output: refitted slopes

    % ax + by + c = 0
    % m2*x - y + {middle_y - (m2*middle_x)} = 0
    a = refitLineStruct(i).refitSlope;
    b = -1;
    c = clusterLineStruct(i).middlePoint(2)-(refitLineStruct(i).refitSlope*clusterLineStruct(i).middlePoint(1));
    refitLineStruct(i).lineABC = [a b c];
end

%% 6) Refit lines - Manhattan World

%% 6-1) 평행한 직선끼리 offset 비교하기 위해, 기울기가 같은 group끼리 모음
sameSlopeGroupNumber_1 = [];
sameSlopeGroupNumber_2 = [];

slope_group1 = refitLineStruct(1).refitSlope; % 첫 번째 그룹의 기울기
for i = 1 : N_GROUP
    if refitLineStruct(i).refitSlope == slope_group1
        sameSlopeGroupNumber_1 = [sameSlopeGroupNumber_1; refitLineStruct(i).groupNumber];
    else sameSlopeGroupNumber_2 = [sameSlopeGroupNumber_2; refitLineStruct(i).groupNumber];
    end
end

%% 6-2) 평행한 두 직선 사이의 거리들 저장
parallelLinesOffset_1 = nchoosek(sameSlopeGroupNumber_1, 2); % nCr, 2개씩 모음
parallelLinesOffset_2 = nchoosek(sameSlopeGroupNumber_2, 2); % nCr, 2개씩 모음
% for slope 1)
for i = 1:size(parallelLinesOffset_1,1)
    % ax + by + c = 0
    a = refitLineStruct(sameSlopeGroupNumber_1(1)).refitSlope; % 어차피 sameSlopeGroupNumber_1 안의 그룹들은 기울기가 같음. 아무거나 상관없음.
    b = -1;
    c1 = refitLineStruct(parallelLinesOffset_1(i,1)).lineABC(3);
    c2 = refitLineStruct(parallelLinesOffset_1(i,2)).lineABC(3);

    parallelLinesOffset_1(i,3) = parallel_lines_offset(a,b,c1,c2); % 평행한 두 직선 사이의 거리
end

% for slope 2)
for i = 1:size(parallelLinesOffset_2,1)
    % ax + by + c = 0
    a = refitLineStruct(sameSlopeGroupNumber_2(1)).refitSlope; % 어차피 sameSlopeGroupNumber_1 안의 그룹들은 기울기가 같음. 아무거나 상관없음.
    b = -1;
    c1 = refitLineStruct(parallelLinesOffset_2(i,1)).lineABC(3);
    c2 = refitLineStruct(parallelLinesOffset_2(i,2)).lineABC(3);

    parallelLinesOffset_2(i,3) = parallel_lines_offset(a,b,c1,c2); % 평행한 두 직선 사이의 거리
end

%% 6-3) 평행한 두 직선 사이의 거리가 PARALLEL_OFFSET_TH 보다 작으면 한 직선으로 합침
%for slope 1)
eliminate_group_slope1 = [];
for i = 1:size(parallelLinesOffset_1,1)
    %평행한 두 직선 사이의 거리가 PARALLEL_OFFSET_TH 보다 작으면 한 직선으로 합침
    if parallelLinesOffset_1(i,3) <= PARALLEL_OFFSET_TH 
        group1_index = parallelLinesOffset_1(i,1); 
        group2_index = parallelLinesOffset_1(i,2); 

        if ismember(group1_index,eliminate_group_slope1) || ismember(group2_index,eliminate_group_slope1) % 한 그룹이라도 eliminate_group_slope1에 있으면
            continue;
        else
            % group1과 group2의 inlierPoints 합치기
            clusterLineStruct(group1_index).inlierPoints = [clusterLineStruct(group1_index).inlierPoints; clusterLineStruct(group2_index).inlierPoints];
            eliminate_group_slope1 = [eliminate_group_slope1 group2_index]; % 제거할 그룹(line) 
        end
    else continue;
    end
end % return: eliminate_group_slope1

% for slope 2)
eliminate_group_slope2 = [];
for i = 1:size(parallelLinesOffset_2,1)
    %평행한 두 직선 사이의 거리가 PARALLEL_OFFSET_TH 보다 작으면 한 직선으로 합침
    if parallelLinesOffset_2(i,3) <= PARALLEL_OFFSET_TH 
        group1_index = parallelLinesOffset_2(i,1);
        group2_index = parallelLinesOffset_2(i,2); 

        if ismember(group1_index,eliminate_group_slope2) || ismember(group2_index,eliminate_group_slope2) % 한 그룹이라도 eliminate_group_slope1에 있으면
            continue;
        else
            % group1과 group2의 inlierPoints 합치기
            clusterLineStruct(group1_index).inlierPoints = [clusterLineStruct(group1_index).inlierPoints; clusterLineStruct(group2_index).inlierPoints];
            eliminate_group_slope2 = [eliminate_group_slope2 group2_index]; % 제거할 그룹(line) 
        end
    else continue;
    end
end % return: eliminate_group_slope2

sameSlopeGroupNumber_1_before_eliminate = sameSlopeGroupNumber_1;
sameSlopeGroupNumber_2_before_eliminate = sameSlopeGroupNumber_2;

% setting
eliminate_group_slope_1and2 = sort([eliminate_group_slope1 eliminate_group_slope2]); % 오름차순 

groupNumber_refitLineStruct = [];
for i = 1:size(refitLineStruct,2)
    groupNumber_refitLineStruct = [groupNumber_refitLineStruct; refitLineStruct(i).groupNumber]; 
end

groupNumber_clusterLineStruct = [];
for i = 1:size(refitLineStruct,2)
    groupNumber_clusterLineStruct = [groupNumber_clusterLineStruct; refitLineStruct(i).groupNumber];
end

if size(eliminate_group_slope_1and2,2) == 0
    disp("");
else
    for i = 1:size(eliminate_group_slope_1and2,2)
        index = find(groupNumber_refitLineStruct == eliminate_group_slope_1and2(i));
        clusterLineStruct(index) = [];
        refitLineStruct(index) = [];
        groupNumber_refitLineStruct(find(groupNumber_refitLineStruct == eliminate_group_slope_1and2(i))) = [];
        groupNumber_clusterLineStruct(find(groupNumber_clusterLineStruct == eliminate_group_slope_1and2(i))) = [];
        
        if ismember(eliminate_group_slope_1and2(i), sameSlopeGroupNumber_1)
            sameSlopeGroupNumber_1(find(sameSlopeGroupNumber_1 == eliminate_group_slope_1and2(i))) = [];
        elseif ismember(eliminate_group_slope_1and2(i), sameSlopeGroupNumber_2)
            sameSlopeGroupNumber_2(find(sameSlopeGroupNumber_2 == eliminate_group_slope_1and2(i))) = [];
        end
    end
end % 이제 한 벽(wall)당 하나의 group(line)만 남겨두고 나머지 group정보는 다 삭제. (inlierPoints만 합치고.)

%% 6-4) refitted orthogonal line --> refitLineStruct.min_max_endpoints
groupNumber_after_refit = [];
for i = 1:size(refitLineStruct,2)
    groupNumber_after_refit = [groupNumber_after_refit; refitLineStruct(i).groupNumber]; % setting
end
% for slope 1)
if abs(refitLineStruct(sameSlopeGroupNumber_1(1)).refitSlope) > 1
    for i = 1:size(sameSlopeGroupNumber_1,1)
        index = find(groupNumber_after_refit == sameSlopeGroupNumber_1(i));
        % min_endpoint 
        y1 = min(clusterLineStruct(index).inlierPoints(:,2))-0.5; % line 늘리기
        x1 = (y1 - refitLineStruct(index).lineABC(3))/refitLineStruct(index).lineABC(1); % x = (y-c)/a
        % max_endpoint
        y2 = max(clusterLineStruct(index).inlierPoints(:,2))+0.5; % line 늘리기
        x2 = (y2 - refitLineStruct(index).lineABC(3))/refitLineStruct(index).lineABC(1);% x = (y-c)/a
        refitLineStruct(index).min_max_endpoints = [x1 y1;x2 y2];
    end
else
    for i = 1:size(sameSlopeGroupNumber_1,1)
        index = find(groupNumber_after_refit == sameSlopeGroupNumber_1(i));
        % min_endpoint
        x1 = min(clusterLineStruct(index).inlierPoints(:,1))-0.5; % line 늘리기
        y1 = refitLineStruct(index).lineABC(1)*x1 + refitLineStruct(index).lineABC(3); % y = ax + c
        % max_endpoint
        x2 = max(clusterLineStruct(index).inlierPoints(:,1))+0.5; % line 늘리기
        y2 = refitLineStruct(index).lineABC(1)*x2 + refitLineStruct(index).lineABC(3); % y = ax + c
        refitLineStruct(index).min_max_endpoints = [x1 y1;x2 y2];
    end
end

% for slope 2)
if abs(refitLineStruct(sameSlopeGroupNumber_2(1)).refitSlope) > 1
    for i = 1:size(sameSlopeGroupNumber_2,1)
        index = find(groupNumber_after_refit == sameSlopeGroupNumber_2(i));
        % min_endpoint 
        y1 = min(clusterLineStruct(index).inlierPoints(:,2))-0.5; % line 늘리기
        x1 = (y1 - refitLineStruct(index).lineABC(3))/refitLineStruct(index).lineABC(1); % x = (y-c)/a
        % max_endpoint
        y2 = max(clusterLineStruct(index).inlierPoints(:,2))+0.5; % line 늘리기
        x2 = (y2 - refitLineStruct(index).lineABC(3))/refitLineStruct(index).lineABC(1);% x = (y-c)/a
        refitLineStruct(index).min_max_endpoints = [x1 y1;x2 y2];
    end
else
    for i = 1:size(sameSlopeGroupNumber_2,1)
        index = find(groupNumber_after_refit == sameSlopeGroupNumber_2(i));
        % min_endpoint
        x1 = min(clusterLineStruct(index).inlierPoints(:,1))-0.5; % line 늘리기
        y1 = refitLineStruct(index).lineABC(1)*x1 + refitLineStruct(index).lineABC(3); % y = ax + c
        % max_endpoint
        x2 = max(clusterLineStruct(index).inlierPoints(:,1))+0.5; % line 늘리기
        y2 = refitLineStruct(index).lineABC(1)*x2 + refitLineStruct(index).lineABC(3); % y = ax + c
        refitLineStruct(index).min_max_endpoints = [x1 y1;x2 y2];
    end
end


%% 6-5) refitLineStruct.intersection_endpoints

% 1) 각 end point끼리 가장 가까운 점 pair 구하기 
% 결과: (unique_nearest_endpoints_pair)

% setting
min_max_endpoints = [];
nearest_endpoints_pair = [];
endpointsNumber = [];
for i = 1:size(refitLineStruct,2)
    min_max_endpoints = [min_max_endpoints; refitLineStruct(i).min_max_endpoints]; % setting
end

for i = 1:size(min_max_endpoints,1) 
    % (1) i 번째 점을 제외한 점 집합 endpoints_except_i 만들기   
    for j = 1:size(min_max_endpoints,1)
        endpoints_except_i(j,:) = min_max_endpoints(j,:);
    end
    endpoints_except_i(i,:) = []; % i번째 행 삭제, endpoint_i 와 비교군
    
    % (2) i 번째 점과 가장 가까운 점 찾기
    % return 1) 'dist_endpoints' (euclidean distance between endpoint_i and the nearest point)
    % return 2) 'nPointIndex_in_min_max_endpoints' (nearest point index in min_max_endpoints)  
    endpoint_i = [min_max_endpoints(i,1),min_max_endpoints(i,2)]; % 전체 min_max_endpoints에서 i 번째 점 (x,y) 정의
    [nPointIndex_in_endpoints_except_i,dist_endpoints] = dsearchn(endpoints_except_i, endpoint_i); 
    nPointIndex_in_min_max_endpoints = find(min_max_endpoints == endpoints_except_i(nPointIndex_in_endpoints_except_i,1)); % min_max_endpoints 행렬에서의 index로 변환
    
    nearest_endpoints_pair = [nearest_endpoints_pair;i nPointIndex_in_min_max_endpoints]; % 결과) 가장 가까운 점 index들 pair
end

% 중복되는 nearest_endpoints_pair 행 제거
for i = 1:size(nearest_endpoints_pair,1)
    sort_nearest_endpoints_pair(i,:) = sort(nearest_endpoints_pair(i,:));
end
unique_nearest_endpoints_pair = unique(sort_nearest_endpoints_pair,"rows"); % 중복 min_max_endpoints 쌍 제거

% 2) unique_nearest_endpoints_pair의 각 endpoint가 어느 line의 endpoint인지 확인하는 작업
% min_max_endpoints 옆에 lineNumber 붙이기 위한 작업 --> min_max_endpoints_lineNumber
groupNumber_double = [groupNumber_after_refit;groupNumber_after_refit];% ex) [2; 3; 4; 5; 6; 7; 2; 3; 4; 5; 6; 7]
groupNumber_double = sort(groupNumber_double); % ex) [2; 2; 3; 3; 4; 4; 5; 5; 6; 6; 7; 7]

min_max_endpoints_lineNumber = [min_max_endpoints groupNumber_double]; % endpoint_x, endpoint_y, line number(=group number)

% 3) 교차점을 찾을 직선 쌍 구하기 (intersect_line_pair)
intersect_line_pair = [];
for i = 1:size(unique_nearest_endpoints_pair,1)
    index1 = unique_nearest_endpoints_pair(i,1); %endpoint1
    index2 = unique_nearest_endpoints_pair(i,2); %endpoint2
    intersect_line_pair = [intersect_line_pair; min_max_endpoints_lineNumber(index1,3) min_max_endpoints_lineNumber(index2,3)]; % line number1, line number2
end

% 4) 교점 구하기
intersecting_points = [];
for i = 1:size(intersect_line_pair,1)
    index1 = find(groupNumber_after_refit == intersect_line_pair(i,1)); % find(groupNumber == line1), refitLineStruct에 해당 groupNumber가 있는 인덱스
    index2 = find(groupNumber_after_refit == intersect_line_pair(i,2)); % find(groupNumber == line2), refitLineStruct에 해당 groupNumber가 있는 인덱스
    
    % line1) y = m1*x + b1
    m1 = refitLineStruct(index1).lineABC(1);
    b1 = refitLineStruct(index1).lineABC(3);
    % line2) y = m2*x + b2
    m2 = refitLineStruct(index2).lineABC(1); 
    b2 = refitLineStruct(index2).lineABC(3);
    
    [x_int, y_int] = line_intersection([m1,b1],[m2,b2]); % intersecting point (x_int, y_int)
    intersecting_points = [intersecting_points; x_int y_int]; % append intersecting points
end
intersecting_twoLines_and_point = [intersect_line_pair intersecting_points]; % line1, line2, x_int, y_int

% 5) 구한 교점은 line 별로 저장하기 (refitLineStruct.intersecting_endpoints)
for i = 1:size(groupNumber_after_refit,1)
    [row,col] = find(intersect_line_pair == groupNumber_after_refit(i));
    intersecting_endpoint_1 = intersecting_twoLines_and_point(row(1),[3:4]); % inetersecting endpoint1 (x,y)
    intersecting_endpoint_2 = intersecting_twoLines_and_point(row(2),[3:4]); % inetersecting endpoint2 (x,y)

    index = find(groupNumber_after_refit == groupNumber_after_refit(i)); % 사실, 그냥 i랑 똑같음
    refitLineStruct(index).intersecting_endpoints = [intersecting_endpoint_1; intersecting_endpoint_2];

%     %% 7) plot 2D floorplan
%     x1_intersecting_endpoint = refitLineStruct(i).intersecting_endpoints(1);
%     y1_intersecting_endpoint = refitLineStruct(i).intersecting_endpoints(3);
%     x2_intersecting_endpoint = refitLineStruct(i).intersecting_endpoints(2);
%     y2_intersecting_endpoint = refitLineStruct(i).intersecting_endpoints(4);
%     
%     m = (y2_intersecting_endpoint - y1_intersecting_endpoint)/(x2_intersecting_endpoint - x1_intersecting_endpoint);
%     x = [min(x1_intersecting_endpoint,x2_intersecting_endpoint) max(x1_intersecting_endpoint,x2_intersecting_endpoint)];
%     y = m*(x-x1_intersecting_endpoint) + y1_intersecting_endpoint;
%     plot(x, y, 'r-', 'LineWidth',2)
%     xlabel('X[m]') 
%     ylabel('Y[m]')
%     set(gcf,'Color','w')
%     axis equal
%     hold on
% 
%     %% 8) plot 3D floorplan
% 
%     x1 = x1_intersecting_endpoint; 
%     y1 = y1_intersecting_endpoint;
%     z1 = 0;
%     
%     % x, y, z of plane 2)
%     x2 = x2_intersecting_endpoint;
%     y2 = y2_intersecting_endpoint;
%     z2 = 0;
% 
%     % x, y, z of plane 3)
%     x3 = x2_intersecting_endpoint;
%     y3 = y2_intersecting_endpoint;
%     z3 = ceiling_height;
% 
%     
%     % x, y, z of plane 4)
%     x4 = x1_intersecting_endpoint; 
%     y4 = y1_intersecting_endpoint; 
%     z4 = ceiling_height;
% 
%     plot_plane(x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4) % plot plane
%     xlabel('X[m]') 
%     ylabel('Y[m]')
%     zlabel('Z[m]')
%     set(gcf,'Color','w')
%     axis equal
    %legend('Points','RANSAC','FloorPlan','Wall');
end

%%
figure;
for i = 1:size(groupNumber_after_refit,1)
    x1_intersecting_endpoint = refitLineStruct(i).intersecting_endpoints(1);
    y1_intersecting_endpoint = refitLineStruct(i).intersecting_endpoints(3);
    x2_intersecting_endpoint = refitLineStruct(i).intersecting_endpoints(2);
    y2_intersecting_endpoint = refitLineStruct(i).intersecting_endpoints(4);
    
    m = (y2_intersecting_endpoint - y1_intersecting_endpoint)/(x2_intersecting_endpoint - x1_intersecting_endpoint);
    x = [min(x1_intersecting_endpoint,x2_intersecting_endpoint) max(x1_intersecting_endpoint,x2_intersecting_endpoint)];
    y = m*(x-x1_intersecting_endpoint) + y1_intersecting_endpoint;
    plot(x, y, 'k-', 'LineWidth',2.5)
    xlabel('X[m]','FontSize',17,'fontname','times new roman') 
    ylabel('Y[m]','FontSize',17,'fontname','times new roman')
    set(gcf,'Color','w')
    set(gca,'FontSize',17,'fontname','times new roman')
    axis equal
    hold on
end

%%
%figure;
for i = 1:size(groupNumber_after_refit,1)
    x1_intersecting_endpoint = refitLineStruct(i).intersecting_endpoints(1);
    y1_intersecting_endpoint = refitLineStruct(i).intersecting_endpoints(3);
    x2_intersecting_endpoint = refitLineStruct(i).intersecting_endpoints(2);
    y2_intersecting_endpoint = refitLineStruct(i).intersecting_endpoints(4);
    
    m = (y2_intersecting_endpoint - y1_intersecting_endpoint)/(x2_intersecting_endpoint - x1_intersecting_endpoint);
    x = [min(x1_intersecting_endpoint,x2_intersecting_endpoint) max(x1_intersecting_endpoint,x2_intersecting_endpoint)];
    y = m*(x-x1_intersecting_endpoint) + y1_intersecting_endpoint;

    x1 = x1_intersecting_endpoint; 
    y1 = y1_intersecting_endpoint;
    z1 = 0;
    
    % x, y, z of plane 2)
    x2 = x2_intersecting_endpoint;
    y2 = y2_intersecting_endpoint;
    z2 = 0;

    % x, y, z of plane 3)
    x3 = x2_intersecting_endpoint;
    y3 = y2_intersecting_endpoint;
    z3 = ceiling_height;

    
    % x, y, z of plane 4)
    x4 = x1_intersecting_endpoint; 
    y4 = y1_intersecting_endpoint; 
    z4 = ceiling_height;

    plot_plane(x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4) % plot plane
    xlabel('X[m]','FontSize',19,'fontname','times new roman') 
    ylabel('Y[m]','FontSize',19,'fontname','times new roman')
    zlabel('Z[m]','FontSize',19,'fontname','times new roman')
    set(gcf,'Color','w')
    set(gca,'FontSize',20,'fontname','times new roman')
    axis equal
    box off
end