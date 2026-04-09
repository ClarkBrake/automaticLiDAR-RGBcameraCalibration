# LiDAR–Camera Calibration Pipeline

This repository contains a MATLAB-based pipeline for performing extrinsic calibration between a LiDAR sensor and a camera using a checkerboard target. The goal is to estimate a rigid transformation that aligns LiDAR point clouds with camera images for sensor fusion tasks.

---

## Overview

The pipeline:

* Detects a checkerboard in camera images
* Extracts the corresponding planar surface from LiDAR point clouds
* Computes geometric features (normals, centroids, basis vectors)
* Estimates the LiDAR-to-camera transformation using SVD
* Optionally visualizes fused LiDAR–camera data

---

## Requirements

* MATLAB (R2022b or newer recommended)
* Computer Vision Toolbox
* Lidar Toolbox

---

## Data Structure

Place your data in the following structure:

output/
├── imagesCalibrationPrimary/   (Calibration images .png)
└── pcdCalibration/            (Corresponding point clouds .pcd)

You also need:

* calibrationSessionCaliPrim.mat (camera calibration file)

---

## How to Run

1. Open MATLAB
2. Navigate to the repository folder
3. Run the main script:

run_calibration

---

## Method Summary

1. Checkerboard Detection (Camera)
   Detects checkerboard corners and reconstructs 3D points

2. ROI Filtering (LiDAR)
   Crops point cloud to a predefined region of interest

3. Plane Extraction
   Uses RANSAC (pcfitplane) to detect the checkerboard plane

4. Geometry Estimation
   Computes surface normals, centroids, and local coordinate frames

5. Alignment (SVD)
   Aligns LiDAR and camera frames using basis vectors and computes rotation + translation

6. Refinement
   Improves alignment using centroid-based correction

---

## Output

The script prints:

* LiDAR-to-camera transformation matrix (4×4)
* Mean normal alignment error (degrees)
* Mean centroid alignment error (mm)

---

## Notes

* At least 3 valid frames are required for calibration
* Ensure good checkerboard visibility across frames
* ROI may need tuning depending on your setup
* Poor alignment usually indicates bad plane detection or insufficient data diversity

---

## Applications

* Sensor fusion (LiDAR + RGB)
* Autonomous systems
* Object detection with depth
* Robotics and perception pipelines

---

## Author

Developed as part of an optical systems and sensor fusion project.
