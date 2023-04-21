# Manhattan world mapping in real time with sparse sensing

## Description
* This code uses a low-cost four-point LiDAR that can only measure four distances per scan to map a structured indoor environment. This code is part of **FL-SLAM**. (This paper has not been published yet.)

<img src="https://user-images.githubusercontent.com/77608922/233629245-58a2a99d-6c0d-4492-9009-d012fbe193ce.PNG" width="700">

* The four-point LiDAR can obtain a significantly smaller amount of range measurements than the typical LiDAR with more than 180 range measurements per scan, resulting in **a SLAM with a sparse sensing problem**. We overcome this challenging problem by effectively utilizing the **Manhattan world (MW) structural regularities** of the indoor environments, and recursive **line RANSAC** to represent detected surrounding walls as orthogonal lines directly modeled as landmarks with insufficient range measurements. 

## Run code
run **main_script_MW_mapping.m**

## Demo
* **2D floor plan**
<img src="https://user-images.githubusercontent.com/77608922/233639847-3d595d61-c61d-4a2a-adb4-06b98b2fb5f2.gif" width="500">

* **3D map**
<img src="https://user-images.githubusercontent.com/77608922/233639889-9f59d130-dc6d-415c-8778-daef9102f1ee.gif" width="500">
