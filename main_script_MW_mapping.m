%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Eunju Jeong 
% Email : eunjujeong178@gmail.com
% 
% input data  : 1) ARKit_6DoF_pose.txt 
%                  (timeStamp, r11, r12, r13, tx[m], r21, r22, r23, ty[m], r31, r32, r33, tz[m])
%               2) 3D_point_cloud.txt
%                  (timeStamp, LiDAR1_x,y,z[m] LiDAR2_x,y,z[m] LiDAR3_x,y,z[m] LiDAR4_x,y,z[m])
%               
% output figure : Manhattan world (MW) mapping in real time (2D floor plan & 3D plane map)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
close all;

%% load 3D point cloud data from the Four-Point LiDAR and ARKit 6-DoF pose from iPhone 12 Pro

% common setting to read text files
delimiter = ' ';
headerlinesIn = 1;

% 1) parse ARKit_Map 6-DoF pose
textFileDir = 'input\ARKit_6DoF_pose.txt';

textPoseData_ARKit = importdata(textFileDir, delimiter, headerlinesIn);
poseTime_ARKit = textPoseData_ARKit.data(:,1).';
poseTime_ARKit = (poseTime_ARKit - poseTime_ARKit(1));
poseData_ARKit = textPoseData_ARKit.data(:,2:13); % r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
timestamp_ARKit = textPoseData_ARKit.data(:,1); % timestamp

% ARKit pose with various 6-DoF pose representations
numPose_ARKit = size(poseData_ARKit,1); % the number of the timestamp
T_gc_ARKit = cell(1,numPose_ARKit); % transformation matrix
stateEsti_ARKit = zeros(6,numPose_ARKit); 
R_gc_ARKit = zeros(3,3,numPose_ARKit); % rotation matrix

for imgIdx = 1:numPose_ARKit 
    % rigid body transformation matrix (4x4)
    T_gc_ARKit{imgIdx} = [reshape(poseData_ARKit(imgIdx,:).', 4, 3).'; [0, 0, 0, 1]];
    
    % state vector and rotation matrix (3x3)
    R_gc_ARKit(:,:,imgIdx) = T_gc_ARKit{imgIdx}(1:3,1:3);
    stateEsti_ARKit(1:3,imgIdx) = T_gc_ARKit{imgIdx}(1:3,4);
    [yaw_ARKit, pitch_ARKit, roll_ARKit] = dcm2angle(R_gc_ARKit(:,:,imgIdx));
    stateEsti_ARKit(4:6,imgIdx) = [roll_ARKit; pitch_ARKit; yaw_ARKit];
end

% 2) parse 3D point cloud data from the Four-Point LiDAR
textFileDir_pointcloud = 'input\3D_point_cloud.txt';

textPointCloudData = importdata(textFileDir_pointcloud, delimiter, headerlinesIn);
pointCloud_numPosex12 = textPointCloudData.data(:,2:13); % LiDAR1(x y z) LiDAR2(x y z) LiDAR3(x y z) LiDAR4(x y z)
numPointCloud = size(pointCloud_numPosex12, 1); 
pointCloud_12xnumPose = zeros(12, numPointCloud); 

for imgIdx = 1:numPointCloud
    pointCloud_12xnumPose(:,imgIdx)=pointCloud_numPosex12(imgIdx,:);
end

%% Four-Point LiDAR manhattan world mapping parameters

%-----------------------------------------
% Users can change these parameters
isInlierPoint_lineRANSAC_th = 0.05;       % unit: [m], Threshold for inlier point
num_inlier_points_th = 40;                % If the number of inlier points of the line generated by line RANSAC is greater than this value, we treat it as a reliable wall
angle_th = 5;                             % unit: [deg], Angle difference threshold with one of the MF axis. We refit the slope of the line to orthogonal to the corresponding axis 
th_distance_between_refitted_line = 0.3;  % unit: [m], If the distance between the line and the newly detected point is within this value, we extend the length the line. 
th_distance_between_endPoint = 0.7;       % unit: [m], If the distance between the endpoint of the line and the newly detected point is less than this value, we extend the length the line.  
parallel_offset_th = 0.5;                 % unit: [m], If the distance between two lines is less than this value, merge into one line
%-----------------------------------------

MW_Map_FPLiDAR_sign = false;    % Whether Manhattan world map is built
used_points_accumulate = [];    % (Inlier points used to genertae the line) + (points used to extending the length of the line)
MW_Map_FPLiDAR = [];            % a struct in which reliable orthogonal walls are stored 
MF_X = [1;0;0]; MF_Y = [0;1;0]; % Manhattan frame

for imgIdx = 1:numPose_ARKit 
    %% Four-Point LiDAR manhattan world mapping 
    
    figure(1); cla;

    %% 1) draw moving 2D trajectory
    p_gc_ARKit_Map = stateEsti_ARKit(1:3,1:imgIdx); 
    % plot(p_gc_ARKit_Map(1,:), p_gc_ARKit_Map(2,:), 'm', 'LineWidth', 2); hold on; grid on; axis equal; % draw 2D trajectory
    plot3(p_gc_ARKit_Map(1,:), p_gc_ARKit_Map(2,:), p_gc_ARKit_Map(3,:), 'm', 'LineWidth', 2); hold on; grid on; axis equal; % draw 3D trajectory
    xlabel('X[m]','FontSize',15,'fontname','times new roman') ;
    ylabel('Y[m]','FontSize',15,'fontname','times new roman');
    zlabel('Z[m]','FontSize',15,'fontname','times new roman');
    set(gcf,'Color','w');
    set(gca,'FontSize',15,'fontname','times new roman');

    %% 2) draw four directions point cloud
    fourPoint_xyz = pointCloud_12xnumPose(:,1:imgIdx);

    % % draw 2D point cloud)
    % plot(fourPoint_xyz(1,:), fourPoint_xyz(2,:), 'b.'); hold on;   % LiDAR 1
    % plot(fourPoint_xyz(4,:), fourPoint_xyz(5,:), 'b.'); hold on;   % LiDAR 2
    % plot(fourPoint_xyz(7,:), fourPoint_xyz(8,:), 'b.'); hold on;   % LiDAR 3
    % plot(fourPoint_xyz(10,:), fourPoint_xyz(11,:), 'b.'); hold on; % LiDAR 4

    % draw 3D point cloud)
    plot3(fourPoint_xyz(1,:), fourPoint_xyz(2,:), fourPoint_xyz(3,:), 'b.'); hold on;    % LiDAR 1
    plot3(fourPoint_xyz(4,:), fourPoint_xyz(5,:), fourPoint_xyz(6,:), 'b.'); hold on;    % LiDAR 2
    plot3(fourPoint_xyz(7,:), fourPoint_xyz(8,:), fourPoint_xyz(9,:), 'b.'); hold on;    % LiDAR 3
    plot3(fourPoint_xyz(10,:), fourPoint_xyz(11,:), fourPoint_xyz(12,:), 'b.'); hold on; % LiDAR 4


    %% 3) draw camera body frame and inertial frame 
    Rgc_current = T_gc_ARKit{imgIdx}(1:3,1:3); % rotational motion
    pgc_current = T_gc_ARKit{imgIdx}(1:3,4); % translational motion
    plot_inertial_frame(0.5); % plot inertial frame 
    plot_body_frame(Rgc_current, pgc_current);
    set(gca,'XDir','reverse'); set(gca,'YDir','reverse'); axis equal; hold on;


    %% 4) Four-Point LiDAR manhattan world mapping
    
    % 4-1) define pointCloud
    pointCloud = [pointCloud_numPosex12(1:imgIdx,1:3); pointCloud_numPosex12(1:imgIdx,4:6); pointCloud_numPosex12(1:imgIdx,7:9); pointCloud_numPosex12(1:imgIdx,10:12)]';
    pointCloud_original = [pointCloud_numPosex12(1:imgIdx,1:3); pointCloud_numPosex12(1:imgIdx,4:6); pointCloud_numPosex12(1:imgIdx,7:9); pointCloud_numPosex12(1:imgIdx,10:12)]';


    % 4-2) draw the four points currently detected by the Four-Point LiDAR   
    current_FPpoints = [];
    for i = 1:4 
        if (pointCloud_numPosex12(imgIdx,(i*3)-2) == 0 && pointCloud_numPosex12(imgIdx,(i*3)-1) == 0)
        else
            current_FPpoints = [current_FPpoints,struct('x',pointCloud_numPosex12(imgIdx,(i*3)-2),'y',pointCloud_numPosex12(imgIdx,(i*3)-1),'z',pointCloud_numPosex12(imgIdx,(i*3)))];
        end
    end
 
    num_FPcurrent_points = size(current_FPpoints,2); 

    for i = 1 : num_FPcurrent_points
        plot3(current_FPpoints(i).x, current_FPpoints(i).y, current_FPpoints(i).z, 'o', 'MarkerSize',8,'MarkerEdgeColor', [0.2 0.2 0.2], 'MarkerFaceColor', [0.6 0.6 0.6]);
        
        % current four point line
        x = [current_FPpoints(i).x p_gc_ARKit_Map(1,imgIdx)];
        y = [current_FPpoints(i).y p_gc_ARKit_Map(2,imgIdx)];
        z = [current_FPpoints(i).z p_gc_ARKit_Map(3,imgIdx)];
        plot3(x,y,z, 'Color',[0.2 0.2 0.2], LineWidth=1.5);
    end


    % 4-3) remove the points used to generate the line 
    if MW_Map_FPLiDAR_sign == true
        
        % accumualted points from imgidx 1 to current
        pointCloud_x = pointCloud(1,:);
        pointCloud_y = pointCloud(2,:);
        pointCloud_z = pointCloud(3,:);
        
        % remove the points used to generate the line
        new_pointCloud_x = setdiff(pointCloud_x,used_points_accumulate(1,:),'stable');
        new_pointCloud_y = setdiff(pointCloud_y,used_points_accumulate(2,:),'stable');
        new_pointCloud_z = setdiff(pointCloud_z,used_points_accumulate(3,:),'stable');
        new_pointCloud = [new_pointCloud_x;new_pointCloud_y;new_pointCloud_z];

        % update the pointCloud (only unused points remain)
        pointCloud = new_pointCloud;
    end


    % 4-4) extending the lenght of the line
    if MW_Map_FPLiDAR_sign == true % if the MW_Map_FPLiDAR is built
        for i = 1:length(MW_Map_FPLiDAR)

            % 1) if the line is orthogonal to the Y-axis of MF
            if MW_Map_FPLiDAR(i).alignment == 'y'

                % find the points corresponding to the line stored in MW_Map_FPLiDAR among the newly observed points
                pointsSameWall = points_same_wall(pointCloud, MW_Map_FPLiDAR(i), th_distance_between_refitted_line, th_distance_between_endPoint);  
                
                % newly observed points corresponding to existing walls
                if isempty(pointsSameWall) == 0
                    
                    xmin = min(pointCloud(1,pointsSameWall));
                    xmax = max(pointCloud(1,pointsSameWall));

                    % update the endpoints of the corresponding line
                    MW_Map_FPLiDAR(i).min_xyz_M = [min(xmin, MW_Map_FPLiDAR(i).min_xyz_M(1)) MW_Map_FPLiDAR(i).min_xyz_M(2) 1.2];
                    MW_Map_FPLiDAR(i).max_xyz_M = [max(xmax, MW_Map_FPLiDAR(i).max_xyz_M(1)) MW_Map_FPLiDAR(i).max_xyz_M(2) 1.2];

                    % remove the points used to stetch the line
                    pointCloud(:,pointsSameWall) = [];

                end
                  
            % 2) if the line is orthogonal to the X-axis of MF
            elseif MW_Map_FPLiDAR(i).alignment == 'x'
                
                % find the points corresponding to the line stored in MW_Map_FPLiDAR among the newly observed points
                pointsSameWall = points_same_wall(pointCloud, MW_Map_FPLiDAR(i), th_distance_between_refitted_line, th_distance_between_endPoint); 

                % newly observed points corresponding to existing walls
                if isempty(pointsSameWall) == 0
    
                    ymin = min(pointCloud(2,pointsSameWall));
                    ymax = max(pointCloud(2,pointsSameWall));

                    % update the endpoints of the corresponding line
                    MW_Map_FPLiDAR(i).min_xyz_M = [MW_Map_FPLiDAR(i).min_xyz_M(1) min(ymin, MW_Map_FPLiDAR(i).min_xyz_M(2)) 1.2];
                    MW_Map_FPLiDAR(i).max_xyz_M = [MW_Map_FPLiDAR(i).max_xyz_M(1) max(ymax, MW_Map_FPLiDAR(i).max_xyz_M(2)) 1.2];

                    % remove the points used to stetch the line
                    pointCloud(:,pointsSameWall) = [];

                end 
            end
        end
    end

    % 4-5) generate new lines (walls) and augment MW_Map_FPLiDAR
    while(true)
 
        if size(pointCloud,2) <= 1 % too few points to perform line RANSAC
            break
        elseif size(pointCloud,2) >= 2
            % do line RANSAC 
            [lineIdx, ~, lineModel] = detect_line_RANSAC(pointCloud, isInlierPoint_lineRANSAC_th); % find inlier points and line model (a,b,c)
             
    
            % while loop terminating condition
            if size(lineIdx,2) < num_inlier_points_th % if the number of inlier points is less than the threshold, then do not build a line model
                break
            end      
    
    
            % the middle point of lineModel generated with line RANSAC
            middle_x = (min(pointCloud(1,lineIdx))+max(pointCloud(1,lineIdx)))/2;
            middle_y = (-lineModel(1)*middle_x - lineModel(3))/lineModel(2);
            middlePoint = [middle_x middle_y];
            
            % angle difference between line and Manhattan frame 
            slope_line = -(lineModel(1)/lineModel(2));

            % 1) orthogonal to the Y-axis of MF
            if angle_between_two_vectors(MF_X, slope_line) < angle_th % [deg]  
             
                xmin = min(pointCloud(1,lineIdx));
                xmax = max(pointCloud(1,lineIdx));
   
                % MW_Map_FPLiDAR struct field
                alignment = 'y'; 
                offset = middlePoint(2); % y coordinate of the middle point
                n = MF_Y; % structure alignment (orthogonal to Y-axis)
                score = size(lineIdx,2);
                max_xy_M = [xmax max(pointCloud(2,lineIdx))]; 
                min_xy_M = [xmin min(pointCloud(2,lineIdx))];  
               
                % aumgent MW_Map_FPLiDAR
                MW_Map_FPLiDAR = [MW_Map_FPLiDAR; struct('alignment', alignment, 'offset', offset, 'n', n, 'score', score, 'indices', [], 'max_xyz_M', max_xy_M, 'min_xyz_M', min_xy_M)];
                MW_Map_FPLiDAR_sign = true;

                % remove the newly detected orthogonal wall if it is the same wall in MW_Map_FPLiDAR
                remove_unnecessary_line
    

            % 2) orthogonal to the X-axis of MF
            else            
                ymin = min(pointCloud(2,lineIdx));
                ymax = max(pointCloud(2,lineIdx));
     
                % MW_Map_FPLiDAR struct field
                alignment = 'x'; 
                offset = middlePoint(1); % x value of the middle point
                n = MF_X; % structure alignment (orthogonal to X-axis)
                score = size(lineIdx,2);
                max_xyz_M = [max(pointCloud(1,lineIdx)) ymax 1.2]; 
                min_xyz_M = [min(pointCloud(1,lineIdx)) ymin 1.2];
    
                % aumgent MW_Map_FPLiDAR
                MW_Map_FPLiDAR = [MW_Map_FPLiDAR; struct('alignment', alignment, 'offset', offset, 'n', n, 'score', score, 'indices', [], 'max_xyz_M', max_xyz_M, 'min_xyz_M', min_xyz_M)];      
                MW_Map_FPLiDAR_sign = true;

                % remove the newly detected orthogonal wall if it is the same wall in MW_Map_FPLiDAR
                remove_unnecessary_line
            end     

            pointCloud(:,lineIdx) = [];


          % define the used_points. It is used_points that remove pointCloud from pointCloud_original.  
            pointCloud_original_x = pointCloud_original(1,:);
            pointCloud_original_y = pointCloud_original(2,:);
            pointCloud_original_z = pointCloud_original(3,:);
            
            used_points_x = setdiff(pointCloud_original_x,pointCloud(1,:),'stable');
            used_points_y = setdiff(pointCloud_original_y,pointCloud(2,:),'stable');
            used_points_z = setdiff(pointCloud_original_z,pointCloud(3,:),'stable');
            used_points = [used_points_x;used_points_y;used_points_z];

            used_points_accumulate = [used_points_accumulate used_points];        
        end       
    end

    plot_floor_plan; %view(0, 90); % 2D floor plan
    plot_plane; view(47, 48); % 3D map
    imgIdx
end 
