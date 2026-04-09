# LiDAR–Camera Calibration using Checkerboard

## Overview

This repository implements a full pipeline for estimating the **extrinsic transformation between a LiDAR sensor and a camera** using a checkerboard target.

The method aligns:

* 3D checkerboard points reconstructed in the **camera frame**
* A planar surface extracted from the **LiDAR point cloud**

The final output is a **rigid transformation (R, t)** that maps LiDAR points into the camera coordinate system.

---

## Features

* Automatic checkerboard detection in images
* 3D reconstruction of checkerboard corners using camera intrinsics
* Plane extraction from LiDAR using RANSAC
* Robust centroid + SVD-based alignment
* Multi-frame aggregation for stable calibration
* Optional LiDAR–camera fusion visualization

---

## Repository Structure

```
main_calibrate_lidar_camera.m        % Main calibration script

estimateCheckerboardCorners3d.m      % Camera-side 3D reconstruction
computeBoundingBox.m                 % 2D checkerboard bounding box helper
extractCheckerboardPlane.m           % LiDAR plane extraction (RANSAC)
getCameraBoardGeometry.m             % Camera checkerboard geometry
getLidarBoardGeometry.m              % LiDAR checkerboard geometry

README.md
```

---

## Requirements

MATLAB with the following toolboxes:

* Computer Vision Toolbox
* Lidar Toolbox

---

## Data Setup

### 1. Directory Structure

You must organize your data as follows:

```
output/
│
├── imagesCalibrationPrimary/
│   ├── frame_000001.png
│   ├── frame_000002.png
│   └── ...
│
├── pcdCalibration/
│   ├── frame_000001.pcd
│   ├── frame_000002.pcd
│   └── ...
```

Each image must correspond to a point cloud with the **same filename index**.

---

### 2. Camera Calibration File

Place your calibration file in the root directory:

```
calibrationSessionCaliPrim.mat
```

This file must contain:

```
calibrationSession.CameraParameters
```

---

### 3. Checkerboard Requirements

* One side must be **even**, the other **odd**
* Square size must match:

```matlab
squareSizeMM = 100;
```

---

## How to Run

1. Open MATLAB
2. Navigate to the repository folder
3. Run:

```matlab
main_calibrate_lidar_camera
```

---

## What the Code Does

### Step 1: Detect Checkerboard

* Finds 2D corners in each image
* Converts them into **3D camera coordinates**

### Step 2: Extract LiDAR Plane

* Crops ROI
* Uses `pcfitplane` (RANSAC)
* Removes dominant ground plane if needed

### Step 3: Estimate Board Geometry

* Computes:

  * Normal vector
  * Centroid
  * Local coordinate axes

### Step 4: Align LiDAR to Camera

* Uses **SVD-based rotation estimation**
* Uses **median centroid alignment for translation**
* Refines transformation using all frames

---

## Output

The script prints:

```
Estimated LiDAR-to-Camera Transform:
[4x4 matrix]
```

And reports:

* Mean normal alignment error (degrees)
* Mean centroid error (mm)

---

## Optional Visualization

If available, the script will run:

```
helperFuseLidarCamera1(...)
```

This overlays LiDAR points onto the image using the estimated transform.

---

## Important Notes

* At least **3 valid frames** are required
* Good checkerboard visibility is critical
* ROI tuning may be required for different setups
* Poor calibration typically results from:

  * Incorrect square size
  * Bad checkerboard detection
  * Noisy LiDAR plane extraction

---

## Future Improvements

* Automatic ROI selection
* Outlier frame rejection
* Nonlinear optimization refinement
* Real-time calibration pipeline

---

## Author

Clark Brake
Carleton University — Optical Systems & Sensors

---

## License

This project is for academic and research use.

