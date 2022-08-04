% -----------------------------------------------------------------------------------------------------------------------
% Original source code : "https://github.com/PyojinKim/ARKit-Data-Logger/tree/master/Visualization"
% 
% Revised for Crazyflie by Eunju Jeong (eunju0316@sookmyung.ac.kr)
% 
% input data  : 2) FlowdeckTime_Optitrack_Crazyflie_6DoF_pose.txt 
%                  (unixTimeStamp, r11, r12, r13, tx[m], r21, r22, r23, ty[m], r31, r32, r33, tz[m])
%               3) globalframe_pointcloud.txt or raw_pointcloud.txt 
%                  (unixTimeStamp up_x,y,z[m] down_x,y,z[m] left_x,y,z[m] right_x,y,z[m] front_x,y,z[m] back_x,y,z[m])
%               
% output figure : 1) Update rate of Crazyflie pose
%                 2) Crazyflie 6DoF motion estimation & 6 directions point cloud result
%                 3) roll[rad] / pitch[rad] / yaw[rad] of Crazsyflie orientation
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

%% 1) parse Crazyflie point cloud data (1x19): Optitrack 

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

%% 2) parse Crazyflie 6 DoF pose data by Optitrack

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
%% 3) plot ARKit VIO results

% 1) play 3D moving trajectory of Crazyflie pose (Figure 10)
figure(10);
for k = 1:numPose_optitrack
    figure(10); cla;

    % draw moving trajectory (optitrack)
    p_gc_CF_optitrack = stateEsti_CF_optitrack(1:3,1:k);
    plot3(p_gc_CF_optitrack(1,:), p_gc_CF_optitrack(2,:), p_gc_CF_optitrack(3,:), 'c', 'LineWidth', 2); hold on; grid on; axis equal;
    
    % draw 6 direction point cloud: Optitrack
    sixpoint_CF_Optitrack = pointcloud_CF_Optitrack(:,1:k);
%     plot3(sixpoint_CF_Optitrack(1,:), sixpoint_CF_Optitrack(2,:), sixpoint_CF_Optitrack(3,:), 'b.'); hold on; % up point (+z)
%     plot3(sixpoint_CF_Optitrack(4,:), sixpoint_CF_Optitrack(5,:), sixpoint_CF_Optitrack(6,:), 'b.'); hold on; % down point (-z)
    plot3(sixpoint_CF_Optitrack(7,:), sixpoint_CF_Optitrack(8,:), sixpoint_CF_Optitrack(9,:), 'b.'); hold on; % left point (+y)
    plot3(sixpoint_CF_Optitrack(10,:), sixpoint_CF_Optitrack(11,:), sixpoint_CF_Optitrack(12,:), 'b.'); hold on; % right point (-y)
    plot3(sixpoint_CF_Optitrack(13,:), sixpoint_CF_Optitrack(14,:), sixpoint_CF_Optitrack(15,:), 'b.'); hold on; % front point (+x)
    plot3(sixpoint_CF_Optitrack(16,:), sixpoint_CF_Optitrack(17,:), sixpoint_CF_Optitrack(18,:), 'b.'); hold on; % back point (-x)
    axis equal;


    % draw camera body and frame (optitrack)
    plot_inertial_frame(0.5); view(47, 48);
    Rgc_CF_current = T_gc_CF_optitrack{k}(1:3,1:3);
    pgc_CF_current = T_gc_CF_optitrack{k}(1:3,4);
    plot_CF_frame(Rgc_CF_current, pgc_CF_current, 0.5, 'm'); hold off;
    refresh; pause(0.01); k
end



% 2) plot Crazyflie motion estimation results (Figure 2)
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
plot_inertial_frame(0.5); legend('Optitrack','Point cloud: Optitrack'); axis equal; view(26, 73);
xlabel('x [m]','fontsize',10); ylabel('y [m]','fontsize',10); zlabel('z [m]','fontsize',10); hold off;


% figure options
f = FigureRotator(gca());