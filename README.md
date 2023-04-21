# Manhattan world mapping in real time with sparse sensing

## Description
* This code uses a low-cost four-point LiDAR that can only measure four distances per scan to map a structured indoor environment. This code is part of **FL-SLAM**. (This paper has not been published yet.)

![Alt text](C:\Users\Eunju Jeong\Desktop\FPLiDAR.PNG)


* The four-point LiDAR can obtain a significantly smaller amount of range measurements than the typical LiDAR with more than 180 range measurements per scan, resulting in **a SLAM with a sparse sensing problem**. We overcome this challenging problem by effectively utilizing the **Manhattan world (MW) structural regularities** of the indoor environments, and recursive **line RANSAC** to represent detected surrounding walls as orthogonal lines directly modeled as landmarks with insufficient range measurements. 

## Run code
run 'main_script_MW_mapping.m'

## Demo
