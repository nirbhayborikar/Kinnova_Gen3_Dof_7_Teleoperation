# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]
### Added
- aarch64 support
- [9d4205a](https://github.com/DavidG-Develop/ros2_kortex/commit/9d4205a2b7334d21dc7ae8d6a67067cb87172d6d)

### Fixed
- PI issue in URDFs
- [9735884](https://github.com/DavidG-Develop/ros2_kortex/commit/9735884bd3f8223e01348ade9310fa5c4d853536)

### Changed
- kortex_driver and urdfs to correctly control the 140 gripper
- Before this change the gripper would not close all the way -> urdf close = ~0.7 real gripper close = ~0.8
- ***Note: This is a hotfix and should be removed when the urdf of the gripper is fixed***
- [050ab89](https://github.com/DavidG-Develop/ros2_kortex/commit/050ab8975e8570d1b0a17203428abb3d091d50dd)