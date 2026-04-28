# Nebula Vendor-Neutral Interface: Remediation Plan (v3)

## 1. High Priority: Absolute ROS Purge from Core
- [x] **Agnostic Driver Base**: Remove `sensor_msgs` include and dependencies from `nebula_driver_base.hpp`.
- [x] **Agnostic Packet Type**: Define `drivers::NebulaPacket` in `nebula_core_common` to replace `nebula_msgs::msg::NebulaPacket` in vendor-neutral layers.
- [x] **Clean CMake**: Remove all `find_package` for ROS packages in `nebula_core_decoders` and `nebula_core_common`.

## 2. High Priority: Fix Continental Plugin
- [x] **normalized Header**: Remove ROS message includes from `continental_plugin.hpp`.
- [x] **SRR520 Fix**: Correct the field mapping for SRR520 (use `range`, `azimuth_angle`, etc.).
- [x] **Agnostic Internals**: Replace `nebula_msgs::msg::NebulaPacket` usage with the new agnostic type or handle internally.

## 3. High Priority: Nebula Studio Restoration
- [x] **CMake Fix**: Import `Boost::filesystem` and `Boost::system` in `nebula_studio/CMakeLists.txt`.
- [x] **Header Fix**: Add missing includes for `Logger` and define `RadarPoint` alias in `pcap_converter.cpp`.
- [x] **Discovery Fallback**: Restore source-directory scanning as a fallback when `COLCON_PREFIX_PATH` is incomplete.
- [x] **Config Fidelity**: Ensure all `extra_params` from Studio JSON are passed to vendor runtimes.

## 4. Medium Priority: Robustness and Performance
- [ ] **PCAP Fragmentation**: Add basic IP fragmentation reassembly or detection in `PcapPacketSource`.
- [x] **Discovery Errors**: Implement proper logging/exceptions for plugin parse and load failures in `SensorRegistry`.
- [x] **Library Loading**: Search for plugin libraries relative to the descriptor and in `LD_LIBRARY_PATH`.
- [x] **Live Concurrency**: Add mutex-based serialization in `LiveTransportGraph` for routing and decoding.
- [ ] **Packet Matching**: Expand `PacketRouter` to support offsets and length checks in signatures.
