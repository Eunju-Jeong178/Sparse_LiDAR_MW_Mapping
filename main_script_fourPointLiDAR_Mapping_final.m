% -----------------------------------------------------------------------------------------------------------------------
% Author : Eunju Jeong (eunju0316@sookmyung.ac.kr) 
% 
% input data  : 1) Crazyflie_6DoF_pose.txt 
%                  (unixTimeStamp, r11, r12, r13, tx[m], r21, r22, r23, ty[m], r31, r32, r33, tz[m])
%               2) globalframe_pointcloud.txt
%                  (unixTimeStamp up_x,y,z[m] down_x,y,z[m] left_x,y,z[m] right_x,y,z[m] front_x,y,z[m] back_x,y,z[m])
%               
% output figure : Crazyflie 6DoF motion estimation 
%                 & 6 directions point cloud & 6 DoF pose of Crazyflie
%                 & MW mapping in real time
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

%% 1.Parse Crazyflie 6 DoF pose data by Optitrack

% parsing Crazyflie(CF) pose data text file 
%textFileDir = 'input\StrayScanner_6DoF_pose_20230102_B2_elevator.txt';
%textFileDir = 'input\StrayScanner_6DoF_pose_20230103_B2_openSpace.txt';
%textFileDi = 'input\StrayScanner_6DoF_pose_20230105_B3.txt';
%textFileDir = 'input\StrayScanner_6DoF_pose_20230105_B2_elevator.txt';
%textFileDir = 'input\StrayScanner_6DoF_pose_TF02_B2_openSpace_01_.txt';
%textFileDir = 'input\StrayScanner_6DoF_pose_newRig_20230103_B2_openSpace_forLPSLAM.txt';
textFileDir = 'input\StrayScanner_6DoF_pose_TF02_20230119_openSpace_02_0.3m~20m.txt';

textPoseData = importdata(textFileDir, delimiter, headerlinesIn);
poseTime = textPoseData.data(:,1).';
poseTime = (poseTime - poseTime(1)) ./ milliSecondToSecond;
poseData = textPoseData.data(:,2:13);

% Crazyflie pose with various 6-DoF pose representations
numPose = size(poseData,1);
T_gc = cell(1,numPose); % initialize
stateEsti = zeros(6,numPose); % initialize
R_gc = zeros(3,3,numPose);
for imgIdx = 1:numPose
    
    % rigid body transformation matrix (4x4)
    T_gc{imgIdx} = [reshape(poseData(imgIdx,:).', 4, 3).'; [0, 0, 0, 1]];
    
    % state vector and rotation matrix
    R_gc(:,:,imgIdx) = T_gc{imgIdx}(1:3,1:3);
    stateEsti(1:3,imgIdx) = T_gc{imgIdx}(1:3,4);
    [yaw, pitch, roll] = dcm2angle(R_gc(:,:,imgIdx));
    stateEsti(4:6,imgIdx) = [roll; pitch; yaw];
end

%% 2.Parse Crazyflie point cloud data (1x19): Optitrack 

% parsing Crazyflie point cloud data text file
%textFileDir_pointcloud = 'input\global_pointcloud_1x13_20230102_B2_elevator.txt';
%textFileDir_pointcloud = 'input\global_pointcloud_1x13_20230103_B2_openSpace.txt'; 
%textFileDir_pointcloud = 'input\global_pointcloud_1x13_20230105_B3.txt';
%textFileDir_pointcloud = 'input\global_pointcloud_1x13_20230105_B2_elevator.txt';
%textFileDir_pointcloud = 'input\global_pointcloud_1x13_TF02_B2_openSpace_01_.txt';
%textFileDir_pointcloud = 'input\global_pointcloud_1x13_newRig_20230103_B2_openSpace_forLPSLAM.txt';
textFileDir_pointcloud = 'input\global_pointcloud_1x13_TF02_20230119_openSpace_02_0.3m~20m.txt';

textCFPointCloudData = importdata(textFileDir_pointcloud, delimiter, headerlinesIn);

% Crazyflie 3D point cloud
pointCloud_xyz = textCFPointCloudData.data(:,2:13); % except for timestamp
numPointCloud = size(pointCloud_xyz, 1);
pointcloud_xyz = zeros(12, numPointCloud); % initialize

for imgIdx = 1:numPointCloud
    pointcloud_xyz(:,imgIdx)=pointCloud_xyz(imgIdx,:);
end

%% 3.Manhattan frame mapping parameters

RANSAC_LINE_INLIER_TH = 0.05;             % [m], 랜덤으로 생성한 직선과 어떤 점과의 거리가 이 값 안에 있으면 inlier point로 취급
NUM_INLIER_POINTS_TH = 40;                % RANSAC으로 생성한 line의 inlier point 개수가 이 값 보다 크면 의미있는 벽으로 취급 --> walls 에 저장할 직선
ANGLE_TH = 20;                            % [deg], MF_X축과의 각도차이가 이 값 이하이면 MF_X축과 평행한 것으로 취급
TH_DISTANCE_BETWEEN_REFITTED_LINE = 0.7;  % [m] % 점과 직선사이의 거리가 이 값 안에 있으면 이 점들을 포함하여 line 늘림 (for visualization)
TH_DISTANCE_BETWEEN_ENDPOINT = 1;         % [m] % TH_DISTANCE_BETWEEN_REFITTED_LINE 안에 있는 점 중에서 선의 끝점과의 거리가 이 값보다 작은 값만 저장해서 늘림 (for visualization)
PARALLEL_OFFSET_TH = 0.2;                 % [m] walls 에 저장된 평행한 두 직선의 거리 차이가 이 값 이하이면 첫 번째 line으로 합침 
%ceiling_height = 2.5; % [m]
isPointInThisWall_th = 0.1;  %[m] state update용. 해당 point가 정말로 그 벽에 해당하는지 판단용

used_points_sign = 0;
MW_Map_sign = 0;

used_points_accumulate = []; % (line 생성에 사용된 inlier points) + (line 늘리는데 사용된 points)

MW_Map = []; % 여기는 이제 전체 walls (MW_Map)

%walls_current = []; % 매번 생성되는 모든 line들 누적

current_points = []; % 현재 시점(imgIdx 즉 k)에서 관측되는 4개의 point

%% Plot trajectory, point cloud, line (moving)

% Define Manhattan frame (MF)
MF_X = [1;0.00000001;0.00000001]; MF_Y = [0.00000001;1;0.00000001]; MF_Z = [0.00000001;0.00000001;1];
MF = [MF_X MF_Y MF_Z];

% 1) play 3D moving trajectory of Crazyflie pose

%for k = 1:numPose
for imgIdx = 1: numPose
    figure(1); cla;

%     %% 1) draw moving trajectory
    p_gc = stateEsti(1:3,1:imgIdx);
    %plot3(p_gc(1,:), p_gc(2,:), p_gc(3,:), 'm', 'LineWidth', 2); hold on; grid on; axis equal;
    plot(p_gc(1,:), p_gc(2,:), 'm', 'LineWidth', 2); hold on; grid on; axis equal;
    xlabel('X[m]','FontSize',15,'fontname','times new roman') ;
    ylabel('Y[m]','FontSize',15,'fontname','times new roman');
    %zlabel('Z[m]','FontSize',15,'fontname','times new roman');
    set(gcf,'Color','w');
    set(gca,'FontSize',15,'fontname','times new roman');

    %% 2) draw 4 direction point cloud
    fourPoint_xyz = pointcloud_xyz(:,1:imgIdx);

    plot(fourPoint_xyz(1,:), fourPoint_xyz(2,:), '.'); hold on; % LiDAR 1
    plot(fourPoint_xyz(4,:), fourPoint_xyz(5,:), '.'); hold on; % LiDAR 2
    plot(fourPoint_xyz(7,:), fourPoint_xyz(8,:), '.'); hold on; % LiDAR 3
    %plot(fourPoint_xyz(10,:), fourPoint_xyz(11,:), '.'); hold on; % LiDAR 4

    axis equal; hold on;
  
    %% 3) draw camera body and frame 
    plot_inertial_frame(0.5); %view(47, 48); % plot inertial frame
    Rgc_current = T_gc{imgIdx}(1:3,1:3); % plot body frame
    pgc_current = T_gc{imgIdx}(1:3,4);
%     plot_body_frame(Rgc_current, pgc_current, 0.5, 'm'); hold on;

    %% 4) Manhattan World Mapping (Line fitting)

  % (timestamp 제외하고) 1x18 --> 1x2 로 변형    
    pointCloud = [pointCloud_xyz(1:imgIdx,1:3); pointCloud_xyz(1:imgIdx,4:6); pointCloud_xyz(1:imgIdx,7:9); pointCloud_xyz(1:imgIdx,10:12)]';
    pointCloud_original = [pointCloud_xyz(1:imgIdx,1:3); pointCloud_xyz(1:imgIdx,4:6); pointCloud_xyz(1:imgIdx,7:9); pointCloud_xyz(1:imgIdx,10:12)]';

%     % LiDAR 4 제거 버전
%     pointCloud = [pointCloud_xyz(1:imgIdx,1:3); pointCloud_xyz(1:imgIdx,4:6); pointCloud_xyz(1:imgIdx,7:9)]';
%     pointCloud_original = [pointCloud_xyz(1:imgIdx,1:3); pointCloud_xyz(1:imgIdx,4:6); pointCloud_xyz(1:imgIdx,7:9)]';

%     % LiDAR 3, 4 제거 버전
%     pointCloud = [pointCloud_xyz(1:imgIdx,1:3); pointCloud_xyz(1:imgIdx,4:6)]';
%     pointCloud_original = [pointCloud_xyz(1:imgIdx,1:3);pointCloud_xyz(1:imgIdx,4:6)]';


    % current LiDAR 1
    current_points = struct('x',pointCloud_xyz(imgIdx,1),'y',pointCloud_xyz(imgIdx,2),'z',pointCloud_xyz(imgIdx,3));
    % current LiDAR 2
    current_points = [current_points, struct('x',pointCloud_xyz(imgIdx,4),'y',pointCloud_xyz(imgIdx,5),'z',pointCloud_xyz(imgIdx,6))];
    % current LiDAR 3
    current_points = [current_points, struct('x',pointCloud_xyz(imgIdx,7),'y',pointCloud_xyz(imgIdx,8),'z',pointCloud_xyz(imgIdx,9))];
    % current LiDAR 4
    current_points = [current_points, struct('x',pointCloud_xyz(imgIdx,10),'y',pointCloud_xyz(imgIdx,11),'z',pointCloud_xyz(imgIdx,12))];
    
    num_current_points = size(current_points,2); 
    for i = 1 : num_current_points
        plot(current_points(i).x, current_points(i).y, 'r*');
    end

    % line을 만드는 데 사용된 point 제거
    if used_points_sign == 0
    else
        % 새로운 pointCloud에서 used_points_accumulate 제거
        pointCloud_x = pointCloud(1,:);
        pointCloud_y = pointCloud(2,:);
        pointCloud_z = pointCloud(3,:);
            
        new_pointCloud_x = setdiff(pointCloud_x,used_points_accumulate(1,:),'stable');
        new_pointCloud_y = setdiff(pointCloud_y,used_points_accumulate(2,:),'stable');
        new_pointCloud_z = setdiff(pointCloud_z,used_points_accumulate(3,:),'stable');
        new_pointCloud = [new_pointCloud_x;new_pointCloud_y;new_pointCloud_z];

        pointCloud = new_pointCloud;
    end
    
    %% Line 늘리기
    if MW_Map_sign == 0 % MW_Map이 아직 생성이 안됐다면
    else
        for i = 1:length(MW_Map)
            if MW_Map(i).alignment == 'y'
                pointsSameWall = PointsSameWall(pointCloud, MW_Map(i), TH_DISTANCE_BETWEEN_REFITTED_LINE, TH_DISTANCE_BETWEEN_ENDPOINT);  
                if isempty(pointsSameWall) ~=0
                    continue;
                elseif isempty(pointsSameWall) == 0
                    
 
                    xmin = min(pointCloud(1,pointsSameWall));
                    xmax = max(pointCloud(1,pointsSameWall));

                    % y는 중요하지 않음
                    MW_Map(i).min_xy_M = [min(xmin, MW_Map(i).min_xy_M(1)) MW_Map(i).min_xy_M(2)];
                    MW_Map(i).max_xy_M = [max(xmax, MW_Map(i).max_xy_M(1)) MW_Map(i).max_xy_M(2)];


                    % pointCloud에서 pointsIdxInThres 제거
                    pointCloud(:,pointsSameWall) = [];

%                     % 각 line에 새로 포함된 point 개수 추가 
%                     MW_Map(i).score = length(pointsIdxInThres); % 다시!
                end
                  

            elseif MW_Map(i).alignment == 'x'
                pointsSameWall = PointsSameWall(pointCloud, MW_Map(i), TH_DISTANCE_BETWEEN_REFITTED_LINE, TH_DISTANCE_BETWEEN_ENDPOINT); 

                if isempty(pointsSameWall) ~=0
                    continue;
                elseif isempty(pointsSameWall) == 0
    
                    ymin = min(pointCloud(2,pointsSameWall));
                    ymax = max(pointCloud(2,pointsSameWall));

                    % x는 중요하지 않음
                    MW_Map(i).min_xy_M = [MW_Map(i).min_xy_M(1) min(ymin, MW_Map(i).min_xy_M(2))];
                    MW_Map(i).max_xy_M = [MW_Map(i).max_xy_M(1) max(ymax, MW_Map(i).max_xy_M(2))];


                    % pointCloud에서 pointsIdxInThres 제거
                    pointCloud(:,pointsSameWall) = [];

                end 
            end
        end
    end

    %% current_points(k)와 MW_Map 의 가장 가까운 line 찾기
    
    if MW_Map_sign == 0 % MW_Map이 아직 생성이 안됐다면
    else
        for k = 1:num_current_points
            if (current_points(k).x == 0 && current_points(k).y == 0 && current_points(k).z == 0) % 무시하는 점
            else
                [dist,Map_index] = nearby_LMs_four_point_LiDAR(current_points(k), MW_Map);
                % 이때 current_points(k)는 해당 MW_Map의 alignment를 따라간다. 필요시 walls 만들기

                if (dist < isPointInThisWall_th)
                    disp("This point is correspoinds to this wall.")
                    % [state, MW_Map] = updateExistingStates_withFourPointLiDAR(state, MW_Map, walls(k),Map_index, optsLPSLAM)
                end
            end
        end
    end


    %% Line 생성(line RANSAC) 및 walls 누적
    while(true)

        % do line RANSAC  
        if size(pointCloud,2) <= 1
            break
        elseif size(pointCloud,2) >= 2
            [lineIdx, ~, lineModel] = detectLineRANSAC(pointCloud, RANSAC_LINE_INLIER_TH); % find inlier points and line model (a,b,c)
             
    
            % while loop terminating condition
            if size(lineIdx,2) < NUM_INLIER_POINTS_TH % if the number of inlier points is less than the threshold, then do not build a line model
                break
            end      
    
    
            % line RANSAC으로 생성한 lineModel의 middle point
            middle_x = (min(pointCloud(1,lineIdx))+max(pointCloud(1,lineIdx)))/2;
            middle_y = (-lineModel(1)*middle_x - lineModel(3))/lineModel(2);
            middlePoint = [middle_x middle_y];
            
    
            % 1) Angle difference between line and Manhattan frame --> refit slope
            slope_line = -(lineModel(1)/lineModel(2));
            if angleBetweenTwo3DVectors(MF_X, slope_line) < ANGLE_TH %[deg]
                refitted_slope_line = MF_X(2)/MF_X(1); % MF X축과 평행관계로 refit
    
                % middlePoint를 지나고 기울기가 refitted_slope_line인 직선으로 변경
                % reffited line: a*x + b*y + c = 0
                a = refitted_slope_line;
                b = -1;
                c = middlePoint(2) - (refitted_slope_line * middlePoint(1));
                refittedLineModel = [a b c];
             
                xmin = min(pointCloud(1,lineIdx));
                xmax = max(pointCloud(1,lineIdx));
    
                x_line = [xmin, xmax]; % x를 제한
                y_line = refitted_slope_line * x_line + (middlePoint(2) - (refitted_slope_line * middlePoint(1)));
    
                % walls struct field
                alignment = 'y'; % Manhattan frame(MF) Y축에 수직
                offset = c/sqrt((refitted_slope_line^2)+1);
                n = MF_Y;
                score = size(lineIdx,2);
                refittedLineModel = refittedLineModel; % 이건 line 늘리기 때 확인용
    %             max_xy_M = [xmax a*xmax+c]; % 사실 양 끝점
    %             min_xy_M = [xmin a*xmin+c]; 
                max_xy_M = [xmax max(pointCloud(2,lineIdx))];
                min_xy_M = [xmin min(pointCloud(2,lineIdx))]; 
               
                % aumgent walls
                MW_Map = [MW_Map; struct('alignment', alignment, 'offset', offset, 'n', n, 'score', score, 'refittedLineModel', refittedLineModel, 'max_xy_M', max_xy_M, 'min_xy_M', min_xy_M)];
                MW_Map_sign = MW_Map_sign + 1;
    
                %walls_current = [walls_current; struct('alignment', alignment, 'offset', offset, 'n', n, 'score', score, 'refittedLineModel', refittedLineModel, 'max_xy_M', max_xy_M, 'min_xy_M', min_xy_M, 'imgIdx', k)];

                % MW_Map 내의 line들 중에서 제거할 것이 있으면 제거하기
                removeUnnecessaryLineInWalls
    
            %%
            else
    
                refitted_slope_line = -1/(MF_X(2)/MF_X(1)); % MF Y축과 평행관계로 refit
    
                % middlePoint를 지나고 기울기가 refitted_slope_line인 직선으로 변경
                % reffited line: a*x + b*y + c = 0
                a = refitted_slope_line;
                b = -1;
                c = middlePoint(2) - (refitted_slope_line * middlePoint(1));
                refittedLineModel = [a b c];
                
                
                ymin = min(pointCloud(2,lineIdx));
                ymax = max(pointCloud(2,lineIdx));
    
                y_line = [ymin, ymax];
                x_line = (y_line - (middlePoint(2) - (refitted_slope_line * middlePoint(1))))/refitted_slope_line;   
    
    
                % walls struct field
                alignment = 'x'; % Manhattan frame(MF) X축에 수직
                offset = c/sqrt((refitted_slope_line^2)+1);
                n = MF_X;
                score = size(lineIdx,2);
                refittedLineModel = refittedLineModel; % 이건 line 늘리기 때 확인용
    %             max_xy_M = [(ymax-c)/a ymax]; % 사실 양 끝점
    %             min_xy_M = [(ymin-c)/a ymin];
                max_xy_M = [max(pointCloud(1,lineIdx)) ymax]; 
                min_xy_M = [min(pointCloud(1,lineIdx)) ymin];
    
    
                % aumgent walls
                MW_Map = [MW_Map; struct('alignment', alignment, 'offset', offset, 'n', n, 'score', score, 'refittedLineModel', refittedLineModel, 'max_xy_M', max_xy_M, 'min_xy_M', min_xy_M)];   
                MW_Map_sign = MW_Map_sign + 1;

                %walls_current = [walls_current; struct('alignment', alignment, 'offset', offset, 'n', n, 'score', score, 'refittedLineModel', refittedLineModel, 'max_xy_M', max_xy_M, 'min_xy_M', min_xy_M, 'imgIdx', k)];
    
                % walls 내의 line들 중에서 제거할 것이 있으면 제거하기
                removeUnnecessaryLineInWalls
    
            end
    
%             % plot line RANSAC results 
%             figure; % figure(2) 대신 이걸로 하면 line 한 개씩 각 단계가 따로따로 그려진다. 단계별로 확인하기 좋음
%             %figure(2);
%             plot(pointCloud(1,:), pointCloud(2,:), 'k*'); hold on; grid on; axis equal;
%             plot(pointCloud(1,lineIdx), pointCloud(2,lineIdx), 'ro'); % 원래는 lineIdx
%             %line(x_line, y_line, 'color', 'm', 'LineWidth', 2);
%     
%             % 여긴 이제 고쳐야 됨. 합친 게 반영 안됐음. walls에 누적하는 족족 그리니까. 지금은 제거랑 별개임.********
%             if angleBetweenTwo3DVectors(MF_X, slope_line) < ANGLE_TH
%                 line(x_line, y_line, 'color', 'g', 'marker','s','LineWidth', 3);
%             else
%                 line(x_line, y_line, 'color', 'r', 'marker','s','LineWidth', 3);
%             end
%             xlabel('X[m]','FontSize',15,'fontname','times new roman') ;
%             ylabel('Y[m]','FontSize',15,'fontname','times new roman');
%             set(gcf,'Color','w');
%             set(gca,'FontSize',15,'fontname','times new roman');
%             hold on;      
            %%        
            axis([ -2.4529    5.5749   -1.1148    5.2168])       

            pointCloud(:,lineIdx) = [];

          % pointCloud_original에서 pointCloud를 제거한 것이 used_points 이다. 여기서 used_points를 정의한다.
            pointCloud_original_x = pointCloud_original(1,:);
            pointCloud_original_y = pointCloud_original(2,:);
            pointCloud_original_z = pointCloud_original(3,:);
            
            used_points_x = setdiff(pointCloud_original_x,pointCloud(1,:),'stable');
            used_points_y = setdiff(pointCloud_original_y,pointCloud(2,:),'stable');
            used_points_z = setdiff(pointCloud_original_z,pointCloud(3,:),'stable');
            used_points = [used_points_x;used_points_y;used_points_z];

            used_points_accumulate = [used_points_accumulate used_points];
            used_points_sign = used_points_sign + 1; % used_points_accumulate가 생성되었다는 뜻
           
        end
        
    end

%     plot_plane; view(47, 48);


    % 여기는 while문을 빠져나오고 아무것도 안 할 때 (RANSAC으로 생성한 line의 inlier point 개수가 TH 미만이어서 walls에 추가도 안하고 plot도 안 함
    plot_2Dmap_line_xyplane
    set(gcf,'Color','w');
    set(gca,'FontSize',15,'fontname','times new roman');

    refresh; pause(0.01); 
    imgIdx
end
