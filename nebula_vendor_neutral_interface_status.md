# Nebula Vendor-Neutral Interface Implementation Status

## Phase 1: Core API
- [x] Implement common packet and output types in `nebula_core_common`
- [x] Implement `SensorDecoderRuntime` and `SensorPlugin` interfaces in `nebula_core_decoders`
- [x] Implement packet/transport requirement types
- [x] Add unit tests for core types and interfaces

## Phase 2: Plugin Registry
- [x] Implement plugin descriptor loading and shared-library loading in `nebula_core_runtime`
- [x] Add unit tests for plugin registry

## Phase 3: Sample Sensor Migration
- [x] Update `nebula_sample_common` with registry descriptor and schemas
- [x] Update `nebula_sample_decoders` to implement `SensorPlugin` and `SensorDecoderRuntime`
- [ ] Update `nebula_sample_hw_interfaces`
- [ ] Update sample ROS wrapper
- [x] Add integration tests for sample sensor

## Phase 4: Generic Packet Sources and Router
- [x] Implement `UdpPacketSource`
- [x] Implement `PcapPacketSource`
- [ ] Implement `CanPacketSource`
- [x] Implement Packet Router
- [x] Add unit tests for packet sources and router

## Phase 5: Runtime Sessions
- [x] Implement `ReplaySessionRunner`
- [ ] Implement `LiveTransportGraph`
- [x] Add integration tests for sessions

## Phase 6: Vendor Adapter Migration
- [x] Velodyne migration
- [ ] Hesai migration
- [ ] Robosense migration
- [ ] Seyond migration
- [ ] Continental ARS548 migration
- [ ] Continental SRR520 migration

## Phase 7: Enforcement and Deprecation
- [ ] Add CI checks for plugin discovery
- [ ] Document new sensor requirements
- [ ] Deprecate direct external use of vendor decoder constructors
