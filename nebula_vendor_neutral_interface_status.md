# Nebula Vendor-Neutral Interface Implementation Status

## Phase 1: Core API
- [x] Implement common packet and output types in `nebula_core_common`
- [x] Implement `SensorDecoderRuntime` and `SensorPlugin` interfaces in `nebula_core_decoders`
- [x] **Absolute ROS Purge**: Core packages are hybrid pure-CMake/ament projects with full `Config.cmake` generation.
- [x] **Agnostic Packet Type**: `drivers::NebulaPacket` replaces internal ROS message usage.
- [x] **Restored Core Tests**: All unit tests restored and buildable in both environments.

## Phase 2: Plugin Registry
- [x] Implement plugin descriptor loading and shared-library loading in `nebula_core_runtime`
- [x] **Robust Auto-Discovery**: Comprehensive search across multiple prefixes and source fallbacks.
- [x] **Robust Library Resolution**: Deep search for package-specific libraries in isolated installs and standard paths.
- [x] Add unit tests for plugin registry

## Phase 3: Sample Sensor Migration
- [x] Update `nebula_sample_common` with registry descriptor and schemas
- [x] Update `nebula_sample_decoders` to implement `SensorPlugin` and `SensorDecoderRuntime`
- [x] **Faithful Configuration**: Correct mapping of expanded `SensorConfiguration`.
- [x] Add integration tests for sample sensor (**Verified in isolated install paths**)

## Phase 4: Generic Packet Sources and Router
- [x] Implement `UdpPacketSource`
- [x] Implement `PcapPacketSource` (**Enhanced**: Robust reassembly, VLANs, fragmentation reassembly with strict IPv4 length and buffer safety bounds)
- [x] Implement `CanPacketSource`
- [x] Implement Packet Router (Payload signature matching)
- [x] Add unit tests for packet sources and router


## Phase 5: Runtime Sessions
- [x] Implement `ReplaySessionRunner` (Synchronous and asynchronous support)
- [x] Implement `LiveTransportGraph` (**Thread-safe serialization**)
- [x] Add integration tests for sessions

## Phase 6: Vendor Adapter Migration (Faithful Runtimes)
- [x] Velodyne migration (Corrected `load_from_file`, faithful config)
- [x] Hesai migration (Correction, model name alignment)
- [x] Robosense migration (Corrected config fields and base class mapping, info/DIFOP support)
- [x] Seyond migration (Expanded config)
- [x] Continental migration (**Normalized output**, hidden ROS dependencies)

## Phase 7: Studio Integration
- [x] **Refactor pcap_converter**: Purely vendor-neutral replay via `ReplaySessionRunner`.
- [x] **Robust Python Discovery**: High-fidelity schema and config resolution from isolated layouts and source fallback.
- [x] **Linkage Fixes**: Corrected Boost and ament export mismatches.
- [ ] Deprecate direct external use of vendor decoder constructors
