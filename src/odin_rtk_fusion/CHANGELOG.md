# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-02-01

### Added
- Initial release
- SLAM odometry and GPS (GGA) data fusion based on GTSAM factor graph optimization
- Support for both ROS1 (Noetic) and ROS2 (Humble/Iron/Jazzy)
- Automatic heading angle alignment estimation
- Initial time offset calibration
- GPS outlier rejection
- iSAM2 incremental optimization
- ENU coordinate output
- Optional projected coordinate output (UTM, CGCS2000, etc.)
- RTK driver example for BG550

### Features
- `fusion_node`: Main fusion node supporting dual ROS versions
- Configuration via YAML file
- Lever arm compensation
- Marginalization for long-term operation
