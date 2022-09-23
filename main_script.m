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

%% * Parse Crazyflie 6 DoF pose data by Optitrack

% parsing Crazyflie(CF) pose data text file 
textFileDir_optitrack = 'input\FlowdeckTime_Optitrack_Crazyflie_6DoF_pose.txt'

textPoseData_optitrack = importdata(textFileDir_optitrack, delimiter, headerlinesIn);
CFPoseTime_optitrack = textPoseData_optitrack.data(:,1).';
CFPoseTime_optitrack = (CFPoseTime_optitrack - CFPoseTime_optitrack(1)) ./ milliSecondToSecond;
CFPoseData_optitrack = textPoseData_optitrack.data(:,2:13);

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
CFPointCloudData_Optitrack = textCFPointCloudData_Optitrack.data(:,2:19); % except for timestamp
numPointCloud_Optitrack = size(CFPointCloudData_Optitrack, 1);
pointcloud_CF_Optitrack = zeros(18, numPointCloud_Optitrack); % initialize

for k = 1:numPointCloud_Optitrack
    pointcloud_CF_Optitrack(:,k)=CFPointCloudData_Optitrack(k,:);
end

%% * Manhattan frame mapping parameters

RANSAC_LINE_INLIER_TH = 0.05; % [m], 랜덤으로 생성한 직선과 어떤 점과의 거리가 이 값 안에 있으면 inlier point로 취급
NUM_INLIER_POINTS_TH = 30;
ANGLE_TH = 20; % [deg], MF_X축과의 각도차이가 이 값 이하이면 MF_X축과 평행한 것으로 취급
TH_DISTANCE_BETWEEN_REFITTED_LINE = 0.3; % [m]
PARALLEL_OFFSET_TH = 0.1; % [m] walls 에 저장된 평행한 두 직선의 거리 차이가 이 값 이하이면 첫 번째 line으로 합침 


used_inlierPts_flag = 0;
walls_flag = 0;

used_inlierPts_x_accumulate = [];
idx_unique_used_inlierPts_accumulate = [];

walls = []; % 여기는 이제 전체 walls (MW_Map)

%% * Plot trajectory, point cloud, line (moving)

% Define Manhattan frame (MF)
MF_X = [1;0.0012;0.000824]; MF_Y = [-0.0012;1;-0.0015]; MF_Z = [-0.000826;0.0015;1];
MF = [MF_X MF_Y MF_Z];

% 1) play 3D moving trajectory of Crazyflie pose
figure(1);
figure(2);
figure(3);

%for k = 1:numPose_optitrack
for k = 1: numPose_optitrack
    figure(1); cla;

    %% 1) draw moving trajectory (optitrack)
    p_gc_CF_optitrack = stateEsti_CF_optitrack(1:3,1:k);
    plot3(p_gc_CF_optitrack(1,:), p_gc_CF_optitrack(2,:), p_gc_CF_optitrack(3,:), 'c', 'LineWidth', 2); hold on; grid on; axis equal;
    xlabel('X[m]','FontSize',15,'fontname','times new roman') ;
    ylabel('Y[m]','FontSize',15,'fontname','times new roman');
    zlabel('Z[m]','FontSize',15,'fontname','times new roman');
    set(gcf,'Color','w');
    set(gca,'FontSize',15,'fontname','times new roman');

    %% 2) draw 6 direction point cloud: Optitrack
    sixpoint_CF_Optitrack = pointcloud_CF_Optitrack(:,1:k);
    %plot3(sixpoint_CF_Optitrack(1,:), sixpoint_CF_Optitrack(2,:), sixpoint_CF_Optitrack(3,:), 'b.'); hold on; % up point (+z)inlierPts_num_accumulate
    %plot3(sixpoint_CF_Optitrack(4,:), sixpoint_CF_Optitrack(5,:), sixpoint_CF_Optitrack(6,:), 'b.'); hold on; % down point (-z)
    plot3(sixpoint_CF_Optitrack(7,:), sixpoint_CF_Optitrack(8,:), sixpoint_CF_Optitrack(9,:), 'b.'); hold on; % left point (+y)
    plot3(sixpoint_CF_Optitrack(10,:), sixpoint_CF_Optitrack(11,:), sixpoint_CF_Optitrack(12,:), 'b.'); hold on; % right point (-y)
    %plot3(sixpoint_CF_Optitrack(13,:), sixpoint_CF_Optitrack(14,:), sixpoint_CF_Optitrack(15,:), 'b.'); hold on; % front point (+x)
    %plot3(sixpoint_CF_Optitrack(16,:), sixpoint_CF_Optitrack(17,:), sixpoint_CF_Optitrack(18,:), 'b.'); hold on; % back point (-x)
    axis equal; hold on;
  
    %% 3) draw camera body and frame (optitrack)
    plot_inertial_frame(0.5); view(47, 48);
    Rgc_CF_current = T_gc_CF_optitrack{k}(1:3,1:3);
    pgc_CF_current = T_gc_CF_optitrack{k}(1:3,4);
    plot_CF_frame(Rgc_CF_current, pgc_CF_current, 0.5, 'm'); hold on;

    %% 4) Manhattan World Mapping (Line fitting)

    % (timestamp 제외하고) 1x18 --> 1x2 로 변형
%     pointCloud = [CFPointCloudData_Optitrack(1:k,7:9); CFPointCloudData_Optitrack(1:k,10:12); CFPointCloudData_Optitrack(1:k,13:15); CFPointCloudData_Optitrack(1:k,16:18)]';
%     pointCloud_original = [CFPointCloudData_Optitrack(1:k,7:9); CFPointCloudData_Optitrack(1:k,10:12); CFPointCloudData_Optitrack(1:k,13:15); CFPointCloudData_Optitrack(1:k,16:18)]';
    
    pointCloud = [CFPointCloudData_Optitrack(1:k,7:9); CFPointCloudData_Optitrack(1:k,10:12)]';
    pointCloud_original = [CFPointCloudData_Optitrack(1:k,7:9); CFPointCloudData_Optitrack(1:k,10:12)]';

    % line을 만드는 데 사용된 point 제거
    if used_inlierPts_flag == 0
    else
        for i = 1:size(unique_used_inlierPts_x_accumulate,2)
            idx_unique_used_inlierPts_accumulate = [idx_unique_used_inlierPts_accumulate find(unique_used_inlierPts_x_accumulate(i)==pointCloud_original(1,:))];
        end
        idx_unique_used_inlierPts_accumulate = unique(idx_unique_used_inlierPts_accumulate);
        pointCloud(:,idx_unique_used_inlierPts_accumulate) = [];
    end
    
    %% Line 늘리기
    if walls_flag == 0 % walls가 아직 생성이 안됐다면
    else
        for i = 1:length(walls)
            if walls(i).alignment == 'y'
                pointsIdxInThres = PointsInThres(pointCloud, walls(i).refittedLineModel, TH_DISTANCE_BETWEEN_REFITTED_LINE);
                  
                if isempty(pointsIdxInThres) ~=0
                    continue;
                elseif isempty(pointsIdxInThres) == 0
 
                    xmin = min(pointCloud(1,pointsIdxInThres));
                    xmax = max(pointCloud(1,pointsIdxInThres));

                    % y는 중요하지 않음
                    walls(i).min_xy_M = [min(xmin, walls(i).min_xy_M(1)) walls(i).min_xy_M(2)];
                    walls(i).max_xy_M = [max(xmax, walls(i).max_xy_M(1)) walls(i).max_xy_M(2)];

                    % pointCloud에서 pointsIdxInThres 제거
                    %pointCloud(:,pointsIdxInThres) = [];
                end
                  

            elseif walls(i).alignment == 'x'
                pointsIdxInThres = PointsInThres(pointCloud, walls(i).refittedLineModel, TH_DISTANCE_BETWEEN_REFITTED_LINE);
                    
                if isempty(pointsIdxInThres) ~=0
                    continue;
                elseif isempty(pointsIdxInThres) == 0
    
                    ymin = min(pointCloud(2,pointsIdxInThres));
                    ymax = max(pointCloud(2,pointsIdxInThres));

                    % x는 중요하지 않음
                    walls(i).min_xy_M = [walls(i).min_xy_M(1) min(ymin, walls(i).min_xy_M(2))];
                    walls(i).max_xy_M = [walls(i).max_xy_M(1) max(ymax, walls(i).max_xy_M(2))];


                    % pointCloud에서 pointsIdxInThres 제거
                    %pointCloud(:,pointsIdxInThres) = [];
                end 
            end
        end
    end
    
    %% Line RANSAC and Walls 누적
    while(true)

        % do line RANSAC  
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

            x_line = [xmin, xmax];
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
            walls = [walls; struct('alignment', alignment, 'offset', offset, 'n', n, 'score', score, 'refittedLineModel', refittedLineModel, 'max_xy_M', max_xy_M, 'min_xy_M', min_xy_M)];
            walls_flag = walls_flag + 1;

            % walls 내의 line들 중에서 제거할 것이 있으면 제거하기
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
            walls = [walls; struct('alignment', alignment, 'offset', offset, 'n', n, 'score', score, 'refittedLineModel', refittedLineModel, 'max_xy_M', max_xy_M, 'min_xy_M', min_xy_M)];   
            walls_flag = walls_flag + 1;

            % walls 내의 line들 중에서 제거할 것이 있으면 제거하기
            removeUnnecessaryLineInWalls

        end

        % plot line RANSAC results 
        %figure; % figure(2) 대신 이걸로 하면 line 한 개씩 각 단계가 따로따로 그려진다. 단계별로 확인하기 좋음
        figure(2);
        plot(pointCloud(1,:), pointCloud(2,:), 'k.'); hold on; grid on; axis equal;
        plot(pointCloud(1,lineIdx), pointCloud(2,lineIdx), 'r.'); % 원래는 lineIdx
        %line(x_line, y_line, 'color', 'm', 'LineWidth', 2);

        % 여긴 이제 고쳐야 됨. 합친 게 반영 안됐음. walls에 누적하는 족족 그리니까. 지금은 제거랑 별개임.********
        if angleBetweenTwo3DVectors(MF_X, slope_line) < ANGLE_TH
            line(x_line, y_line, 'color', 'g', 'marker','s','LineWidth', 3);
        else
            line(x_line, y_line, 'color', 'r', 'marker','s','LineWidth', 3);
        end
        xlabel('X[m]','FontSize',15,'fontname','times new roman') ;
        ylabel('Y[m]','FontSize',15,'fontname','times new roman');
        set(gcf,'Color','w');
        set(gca,'FontSize',15,'fontname','times new roman');
        hold on;      
        %%        
        axis([ -2.4529    5.5749   -1.1148    5.2168])       
            
        used_inlierPts_x = [pointCloud(1,lineIdx)]; % 원래는 lineIdx % line 만드는데 사용된 points x좌표 (inlier points)
        used_inlierPts_x_accumulate = [used_inlierPts_x_accumulate used_inlierPts_x];
        unique_used_inlierPts_x_accumulate = unique(used_inlierPts_x_accumulate);
        used_inlierPts_flag = used_inlierPts_flag + 1;

        pointCloud(:,lineIdx) = [];
    end

    % 여기는 while문을 빠져나오고 아무것도 안 할 때 (RANSAC으로 생성한 line의 inlier point 개수가 TH 미만이어서 walls에 추가도 안하고 plot도 안 함

    % 원래 찍히는 point cloud (original)
    %figure(2);
    figure(3);
    plot(pointCloud_original(1,:), pointCloud_original(2,:), 'b.'); hold on; grid on; axis equal;
    plot_2Dmap_line_xyplane
    xlabel('X[m]','FontSize',15,'fontname','times new roman') ;
    ylabel('Y[m]','FontSize',15,'fontname','times new roman');
    set(gcf,'Color','w');
    set(gca,'FontSize',15,'fontname','times new roman');

    refresh; pause(0.01); k
end
