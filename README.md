# Manhattan world mapping in real time with sparse sensing

## Description
* This code uses a low-cost four-point LiDAR that can only measure four distances per scan to map a structured indoor environment. This code is part of **FL-SLAM**. (This paper has not been published yet.)

<img src="https://user-images.githubusercontent.com/77608922/233629245-58a2a99d-6c0d-4492-9009-d012fbe193ce.PNG" width="700">

* The four-point LiDAR can obtain a significantly smaller amount of range measurements than the typical LiDAR with more than 180 range measurements per scan, resulting in **a SLAM with a sparse sensing problem**. We overcome this challenging problem by effectively utilizing the **Manhattan world (MW) structural regularities** of the indoor environments, and recursive **line RANSAC** to represent detected surrounding walls as orthogonal lines directly modeled as landmarks with insufficient range measurements. 

## Input data
* ARKit_6DoF_pose.txt
    timestamp, r_11, r_12, r_13, t_x, r_21, r_22, r_23, t_y, r_31, r_32, r_33, t_z
    
* 3D_point_cloud.txt
    timestamp, point1_x, point1_y, point1_z, point2_x, point2_y, point2_z, point3_x, point3_y, point3_z, point4_x, point4_y, point4_z

## Run code
run **main_script_MW_mapping.m**

## Demos
* **2D floor plan**
<img src="https://user-images.githubusercontent.com/77608922/233639847-3d595d61-c61d-4a2a-adb4-06b98b2fb5f2.gif" width="500">

* **3D map**
<img src="https://user-images.githubusercontent.com/77608922/233639889-9f59d130-dc6d-415c-8778-daef9102f1ee.gif" width="500">
