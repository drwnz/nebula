# Nebula Vendor-Neutral Interface Implementation Status

## Phase 1: Core API
- [x] Implement common packet and output types in `nebula_core_common`
- [x] Implement `SensorDecoderRuntime` and `SensorPlugin` interfaces in `nebula_core_decoders`
- [x] **Absolute ROS Purge**: Removed `sensor_msgs` from core decoders. Agnostic `NebulaPacket` defined.
- [x] **Normalized Payloads**: Defined `RadarDetectionList` and other agnostic types.
- [x] Add unit tests for core types and interfaces

## Phase 2: Plugin Registry
- [x] Implement plugin descriptor loading and shared-library loading in `nebula_core_runtime`
- [x] **Robust Auto-Discovery**: Support for `COLCON_PREFIX_PATH` and `NEBULA_PLUGINS_PATH` traversal with source fallback.
- [x] **Robust Library Loading**: Relative path resolution and better error reporting.
- [x] Add unit tests for plugin registry

## Phase 3: Sample Sensor Migration
- [x] Update `nebula_sample_common` with registry descriptor and schemas
- [x] Update `nebula_sample_decoders` to implement `SensorPlugin` and `SensorDecoderRuntime`
- [x] **Faithful Configuration**: Support for expanded `SensorConfiguration`.
- [x] Add integration tests for sample sensor

## Phase 4: Generic Packet Sources and Router
- [x] Implement `UdpPacketSource`
- [x] Implement `PcapPacketSource` (Robust parsing: VLANs, length checks)
- [x] Implement `CanPacketSource`
- [x] Implement Packet Router (**Enhanced**: Payload signature matching, multi-requirement support)
- [x] Add unit tests for packet sources and router

## Phase 5: Runtime Sessions
- [x] Implement `ReplaySessionRunner` (**Synchronous support** for Studio)
- [x] Implement `LiveTransportGraph` (**Thread-safe**, TCP/HTTP/CAN flexible)
- [x] Add integration tests for sessions

## Phase 6: Vendor Adapter Migration (Faithful Runtimes)
- [x] Velodyne migration (Calibration, FOV, rotation)
- [x] Hesai migration (Correction, model name alignment)
- [x] Robosense migration (Info/DIFOP, signatures)
- [x] Seyond migration (Expanded config)
- [x] Continental ARS548 migration (Normalized radar output, ROS-free header)
- [x] Continental SRR520 migration (Corrected field mapping, ROS-free header)

## Phase 7: Studio Integration
- [x] **Refactor pcap_converter**: Replaced bespoke switches with `ReplaySessionRunner` and `SensorRegistry`.
- [x] **Python Discovery**: Descriptors with source fallback and wrapper package config support.
- [x] **CMake Fix**: Corrected Boost dependencies and include paths.
- [ ] Deprecate direct external use of vendor decoder constructors
