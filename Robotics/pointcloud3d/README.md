# PointCloud3D — Kinect v2 inspection pipeline

A standalone ROS 2 Humble project for Kinect v2 RGB-D acquisition and interactive
3D inspection in RViz, developed as part of my welding-perception workflow.

## Implementation

The Docker-based pipeline uses libfreenect2 with OpenGL GPU depth processing,
organized colored point clouds, an adjustable 3D crop box, depth-validity and
registered RGB selection overlays, and click-selected planar surface fitting.
Accepted measurements can be saved as PLY clouds or planar-face STL meshes with
source-frame and measurement metadata.

## Engineering and validation

The work includes 19 Kinect regression tests, synthetic ROS integration checks,
live RGB-D/validity probes, and repeatable frame-delivery diagnostics. Resolving
undersized DDS shared-memory buffers improved received cloud rate from 1.36 Hz
to 30.00 Hz in a 15-second hardware test; a longer repeat delivered 27.90 Hz.
Later acquisition also ran at 15 Hz, so these are local measurements rather than
a guaranteed 30 FPS specification.

This stage models a selected observed planar patch. Full-object reconstruction,
semantic recognition, multi-view fusion, reference-STL comparison, and calibrated
metrology remain future work.

## Source and reproduction

- [pointcloud3d repository](https://github.com/YoussefSalem05/pointcloud3d)
- [Setup and usage](https://github.com/YoussefSalem05/pointcloud3d#readme)
- [ROS package and API](https://github.com/YoussefSalem05/pointcloud3d/tree/main/ros2_ws/src/kinect_scan)
- [Performance investigation](https://github.com/YoussefSalem05/pointcloud3d/blob/main/docs/implementation/KINECT_PERFORMANCE.md)
