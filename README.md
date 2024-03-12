# Manhattan world mapping in real time with sparse sensing

## Description
* This code uses a low-cost four-point LiDAR that can only measure four distances per scan to map a structured indoor environment. This code is part of **FL-SLAM**. **(RA-L publised 2023.09, "Linear Four-Point LiDAR SLAM for Manhattan World Environments", Eunju Jeong, Jina Lee, Suyoung Kang, and Pyojin Kim)**

* "Linear Four-Point LiDAR SLAM for Manhattan World Environments" (RA-L)
    https://ieeexplore.ieee.org/document/10250905

* Users can change the mapping parameters in main_script_MW_mapping.m according to the input data.

<img src="https://user-images.githubusercontent.com/77608922/233629245-58a2a99d-6c0d-4492-9009-d012fbe193ce.PNG" width="700">

* The four-point LiDAR can obtain a significantly smaller amount of range measurements than the typical LiDAR with more than 180 range measurements per scan, resulting in **a SLAM with a sparse sensing problem**. We overcome this challenging problem by effectively utilizing the **Manhattan world (MW) structural regularities** of the indoor environments, and recursive **line RANSAC** to represent detected surrounding walls as orthogonal lines directly modeled as landmarks with insufficient range measurements. 

## Input data
    1) ARKit_6DoF_pose.txt
    timestamp, r_11, r_12, r_13, t_x, r_21, r_22, r_23, t_y, r_31, r_32, r_33, t_z
    
    2) 3D_point_cloud.txt
    timestamp, point1_x, point1_y, point1_z, point2_x, point2_y, point2_z, point3_x, point3_y, point3_z, point4_x, point4_y, point4_z

## Run code
    run main_script_MW_mapping.m


## Demo
* **2D floor plan**
![2](https://github.com/Eunju-Jeong178/Sparse_LiDAR_MW_Mapping/assets/77608922/3cc594c4-7abb-4adb-94aa-d6c09c8d2f27)



* **3D map**
![3](https://github.com/Eunju-Jeong178/Sparse_LiDAR_MW_Mapping/assets/77608922/36463d8b-768e-4942-8b6e-d680413d0d82)


