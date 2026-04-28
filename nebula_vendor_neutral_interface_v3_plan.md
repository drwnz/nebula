# Nebula Vendor-Neutral Interface: Remediation Plan (v3)

## 1. High Priority: Absolute ROS Purge from Core
- [x] **Agnostic Driver Base**: Removed `sensor_msgs` from `nebula_driver_base.hpp`.
- [x] **Agnostic Packet Type**: Defined `drivers::NebulaPacket` in `nebula_core_common`.
- [x] **Pure CMake Support**: Conditional `ament` usage and standard `Config.cmake` generation in all core packages.

## 2. High Priority: Fix Continental Plugin
- [x] **normalized Header**: Public headers are ROS-free.
- [x] **SRR520 Fix**: Corrected field mapping for SRR520.
- [x] **Agnostic Internals**: Using agnostic types and `Impl` pattern to hide ROS dependencies.

## 3. High Priority: Nebula Studio Restoration
- [x] **CMake Fix**: Imported `Boost::filesystem` and `Boost::system`.
- [x] **Header Fix**: Corrected includes and defined `RadarPoint` alias.
- [x] **Robust Discovery**: Combined descriptor-based and source-directory scanning for schemas/configs.
- [x] **Config Fidelity**: Correctly passing all `extra_params`.

## 4. Medium Priority: Robustness and Performance
- [x] **PCAP Fragmentation**: Implemented full IP reassembly with safety bounds.
- [x] **Discovery Errors**: Improved logging and diagnostic output in `SensorRegistry`.
- [x] **Library Loading**: Deep search in `COLCON_PREFIX_PATH` and descriptor-relative paths.
- [x] **Live Concurrency**: Added mutex serialization in `LiveTransportGraph`.
- [ ] **Packet Matching**: Expand `PacketRouter` with offsets and masks (Deferred).
