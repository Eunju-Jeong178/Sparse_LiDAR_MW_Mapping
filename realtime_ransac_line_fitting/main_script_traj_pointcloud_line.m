% -----------------------------------------------------------------------------------------------------------------------
% Original source code : "https://github.com/PyojinKim/ARKit-Data-Logger/tree/master/Visualization"
% 
% Revised by Eunju Jeong (eunju0316@sookmyung.ac.kr)
% 
% input data  : 1) Crazyflie_6DoF_pose.txt 
%                  (unixTimeStamp, r11, r12, r13, tx[m], r21, r22, r23, ty[m], r31, r32, r33, tz[m])
%               2) globalframe_pointcloud.txt
%                  (unixTimeStamp up_x,y,z[m] down_x,y,z[m] left_x,y,z[m] right_x,y,z[m] front_x,y,z[m] back_x,y,z[m])
%               
% output figure : Crazyflie 6DoF motion estimation 
%                 & 6 directions point cloud 
%                 & Wall clustering(Hierarchical clustering) and Line fitting(RANSAC)
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

%% 1) Parse Crazyflie 6 DoF pose data by Optitrack

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

%% 2) Parse Crazyflie point cloud data (1x19): Optitrack 

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

%% 3) Sparse_MW_Mapping parameter

N_GROUP = 4; % the number of cluster for HC
MAX_DISTANCE_RANSAC = 0.01; % max allowable distance for inliers (positive scalar)

%% 4) Plot trajectory, point cloud, line (moving)

% 1) play 3D moving trajectory of Crazyflie pose (Figure 10)
figure(10);
for k = 1:numPose_optitrack
    figure(10); cla;

    %% 4-1) draw moving trajectory (optitrack)
    p_gc_CF_optitrack = stateEsti_CF_optitrack(1:3,1:k);
    plot3(p_gc_CF_optitrack(1,:), p_gc_CF_optitrack(2,:), p_gc_CF_optitrack(3,:), 'c', 'LineWidth', 2); hold on; grid on; axis equal;

    %% 4-2) draw 6 direction point cloud: Optitrack
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

    %% 4-3) MW_mapping (Application of ICCAS 2022)
   
    %% 4-3-1) parameter 설정
    % (timestamp 제외하고) 1x18 --> 1x2 로 변형
    points = [CFPointCloudData_Optitrack(1:k,7:8); CFPointCloudData_Optitrack(1:k,10:11); CFPointCloudData_Optitrack(1:k,13:14); CFPointCloudData_Optitrack(1:k,16:17)];
    %--------------
    % x,y (left)
    % x,y (right)
    % x,y (front)
    % x,y (back) 
    % 사실 이제 points 에서는 어느 방향인지 상관없음
    %--------------
    numPoints = size(points,1); % the number of 2D point(x,y)
    save pointcloud_1x2.mat points; % save 2D point cloud as mat file

    rng('default'); % For reproducibility
    load pointcloud_1x2.mat

    %% 4-3-2) Hierarchical clustering
    Z = linkage(points,'ward'); % Ward's method
    %dendrogram(Z) % 여기서 이걸 하면 이 figure가 k개만큼 생성된다.. for문 밖으로 빠져나왔을 때 넣기 
    T = cluster(Z,'Maxclust',N_GROUP); % 최대 N_GROUP개의 그룹 % 행(가로축): index of points, 값: cluster group number
    gscatter(points(:,1),points(:,2),T) % 그룹별로 색이 지정된 마커를 사용하여 산점도 Plot 그리기
    set(gcf,'Color','w')
    xlabel('X[m]','FontSize',17,'fontname','times new roman')
    ylabel('Y[m]','FontSize',17,'fontname','times new roman')
    set(gca,'FontSize',17,'fontname','times new roman')
    axis equal
    box off
    hold on;
    %figure;

    cluster_all_index = zeros(numPoints,N_GROUP); %initialization
    for i = 1:numPoints
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
            points_cluster(j,:) = [points(cluster_index(j),1) points(cluster_index(j),2)]; % convert index number to x,y 2D points
        end
    
        % struct 구조로 cluster 정보 저장
        clusterPointStruct(i).groupNumber = i; % cluster group number
        clusterPointStruct(i).pointsIndex = cluster_index; % 그 그룹에 들어있는 점들의 인덱스 (points_without_noise 에서의 인덱스)
        clusterPointStruct(i).numPointsInGroup = num_cluster_index; % 그 그룹에 들어있는 점들의 개수
        clusterPointStruct(i).pointsXY = points_cluster; % 그 그룹에 들어있는 점들의 x,y 좌표 (points_without_noise 에서의 인덱스)
       
        if num_cluster_index > 5
            %% RANSAC - line fitting 
            % Figure 4)
            %plot_ransac(points_cluster) % cluster 된 points 그룹끼리 RANSAC (line fitting) & plane 으로 만들어줌
            
            save pointcloud_cluster_1x2.mat points_cluster;
            load pointcloud_cluster_1x2.mat
        
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
            plot(x, y, 'k-', 'LineWidth',2) % RANSAC (line fitting)
            hold on;
        else
            continue
        end
    end
%%
    refresh; pause(0.01); k

end


%% 5) Plot Results (trajectory and point cloud)

% plot Crazyflie motion estimation results (Figure 2)
figure;

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

gscatter(points(:,1),points(:,2),T) % 그룹별로 색이 지정된 마커를 사용하여 산점도 Plot 그리기
set(gcf,'Color','w')
xlabel('X[m]','FontSize',17,'fontname','times new roman')
ylabel('Y[m]','FontSize',17,'fontname','times new roman')
set(gca,'FontSize',17,'fontname','times new roman')
axis equal; box off; 
hold off;

% figure options
f = FigureRotator(gca());
