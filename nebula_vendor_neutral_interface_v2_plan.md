# Nebula Vendor-Neutral Interface: Remediation and Completion Plan (v2)

## 1. High Priority: ROS-Free Enforcement
- [x] **Purge `sensor_msgs`**: Remove `sensor_msgs` from `nebula_core_decoders` dependencies and all generic interface headers.
- [x] **Normalize Payloads**: Replace ROS-message payloads in `std::any` with ROS-agnostic types (e.g., `NebulaPointCloudPtr` for LiDAR, new `RadarDetectionList` for Radar).
- [x] **Continental Refactor**: Update Continental runtimes to emit normalized radar detections instead of `continental_msgs`.

## 2. High Priority: Advanced Plugin Discovery
- [x] **Prefix Traversal**: Update `SensorRegistry` to automatically scan `COLCON_PREFIX_PATH` for descriptors.
- [x] **Standardized Search**: Search for `share/*/nebula_plugin.json` patterns.
- [x] **Environment Support**: Implement `NEBULA_PLUGINS_PATH` for manual plugin overrides.
- [x] **Factory Symbol Resolution**: Use the `factory` string from the JSON descriptor to resolve the entry point via `dlsym`.

## 3. High Priority: Faithful Vendor Runtimes & Config
- [x] **Expand `SensorConfiguration`**: Include calibration paths, return modes, FOV, rotation speed, and a generic parameter map.
- [x] **Calibration Resolution**: Implement real calibration file loading in all vendor plugins.
- [x] **Hesai Model Fix**: Align descriptor model names with the internal parser spellings.
- [x] **Feature Parity**: Ensure Robosense info/DIFOP, Seyond live/control, and SRR520 CAN IDs are correctly handled.

## 4. High Priority: Studio Integration
- [x] **Studio Converter Refactor**: Replace bespoke decoder switches in `pcap_converter.cpp` with `ReplaySessionRunner`.
- [x] **Studio Discovery**: Update Python-based discovery to utilize `SensorRegistry` logic or descriptors.

## 5. Medium Priority: Robust HW Interfaces
- [x] **Robust PCAP Parsing**: Implement length checks, VLAN support, and robust header extraction in `PcapPacketSource`.
- [x] **Enhanced Routing**: Add payload signature matching to `PacketRouter` to distinguish traffic on the same port/ID.
- [x] **Flexible Graph**: Expand `LiveTransportGraph` to support TCP/HTTP and configurable CAN interfaces.

## 6. Validation
- [ ] **Cross-Workspace Tests**: Verify discovery works from an installed Nebula workspace.
- [ ] **ROS-Free build**: Build `nebula_core_runtime` and `nebula_studio` in a container without ROS environment.
