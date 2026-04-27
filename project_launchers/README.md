# RX2 Urban Eval v1

This folder contains the project-local launch and calibration assets for the RX2 urban short-range LiDAR setup.

## Files

- [rx2_urban_eval_v1.xml](rx2_urban_eval_v1.xml)
- [robinw_test.xml](robinw_test.xml)
- [record_rx2_urban_eval_v1.sh](record_rx2_urban_eval_v1.sh)
- `calibration/`

## Sensor Set

The launch file starts these sensors, each in its own namespace:

- `at128`: Hesai `PandarAT128`
- `ftx140`: Hesai `FTX140`
- `ftx180`: Hesai `FTX180`
- `ot128`: Hesai `PandarQT128`
- `robinw`: Seyond `RobinW`
- `hummingbirdd1`: Seyond `HummingbirdD1`
- `e1`: Robosense `E1`

## Launch Procedure

Source ROS and the workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Launch the full stack in live sensor mode:

```bash
ros2 launch project_launchers/rx2_urban_eval_v1.xml
```

Launch in offline packet replay mode:

```bash
ros2 launch project_launchers/rx2_urban_eval_v1.xml online_sensor:=false
```

The offline mode disables hardware setup and brings the wrappers up ready to subscribe to packet topics from rosbag replay.

Launch only the RobinW sensor with the RX2 defaults:

```bash
ros2 launch project_launchers/robinw_test.xml
```

In live mode, Seyond sensors use calibration fetched from the sensor. If the configured calibration file path does not exist, the fetched calibration is saved there for offline replay.

Launch only the RobinW decoder for packet replay:

```bash
ros2 launch project_launchers/robinw_test.xml online_sensor:=false \
  robinw_calibration_file:=$PWD/project_launchers/calibration/RobinW/anglehv_table.bin
```

## Parameter Structure

The file uses a flat per-sensor argument scheme. Each sensor has its own argument prefix, which keeps overrides predictable.

Common pattern:

- `<sensor>_namespace`
- `<sensor>_config_file`
- `<sensor>_frame_id`
- `<sensor>_host_ip`
- `<sensor>_sensor_ip`

Vendor-specific network/calibration arguments are also exposed:

- Hesai:
  - `<sensor>_correction_file` for `at128`, `ftx140`, `ftx180`
  - `<sensor>_calibration_file` for `ot128`
  - `<sensor>_multicast_ip`
  - `<sensor>_data_port`
  - `<sensor>_gnss_port`
  - `<sensor>_udp_only`
- Seyond:
  - `<sensor>_netmask`
  - `<sensor>_gateway`
  - `<sensor>_udp_port`
  - `<sensor>_udp_message_port`
  - `<sensor>_udp_status_port`
  - `<sensor>_calibration_file`
- Robosense:
  - `e1_launch_hw`
  - `e1_data_port`
  - `e1_gnss_port`
  - `e1_diag_span`

Global mode switch:

- `online_sensor:=true|false`

Behavior:

- `online_sensor:=true` launches live hardware mode
- `online_sensor:=false` disables hardware startup and uses packet-topic replay mode
- `e1_launch_hw` can override the Robosense online/offline state independently if needed

## Calibration Layout

Calibration files live under `project_launchers/calibration/`.

Current structure:

- `AT128/`
- `FTX140/`
- `FTX180/`
- `OT128/`
- `RobinW/`
- `Hummingbird/`
- `E1X/`

Important model behavior:

- Hesai `PandarAT128`, `FTX140`, and `FTX180` use `correction_file`
- Hesai `PandarQT128` uses `calibration_file`
- Seyond sensors use `calibration_file`
- Robosense `E1` uses its normal YAML config and network overrides

For the FTX sensors, both of these are kept:

- base files such as `ftx140.dat` and `ftx180.dat`
- sensor-derived files such as `ftx140_from_sensor_192.168.10.140.dat`

This is intentional:

- offline mode needs the configured base file path to exist
- live mode can prefer the `_from_sensor_<ip>` file when a real sensor has already provided calibration

## Topic Layout

The launch uses namespaces, so topics are grouped by sensor.

Expected packet topics:

- Hesai: `/<namespace>/pandar_packets`
- Seyond: `/<namespace>/seyond_packets`
- Robosense: `/<namespace>/robosense_packets`
- Robosense info: `/<namespace>/robosense_info_packets`

Expected decoded pointcloud topics:

- Hesai: `/<namespace>/pandar_points`
- Seyond: `/<namespace>/seyond_points`
- Robosense: `/<namespace>/robosense_points`

Additional pointcloud outputs where supported:

- Hesai: `/<namespace>/aw_points`, `/<namespace>/aw_points_ex`
- Robosense: `/<namespace>/aw_points`, `/<namespace>/aw_points_ex`

## Rosbag Recording Methodology

Use the helper script to record all relevant packet and pointcloud topics:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./project_launchers/record_rx2_urban_eval_v1.sh
```

Optional output path:

```bash
./project_launchers/record_rx2_urban_eval_v1.sh /data/bags/rx2_urban_eval_v1_run01
```

The recorder script:

- captures raw packet topics and decoded pointcloud topics
- includes unpublished topics so subscriptions are created even if a sensor is not active yet
- splits the bag every `60` seconds with `--max-bag-duration 60`
- relies on rosbag2’s normal split-bag behavior, which produces one `metadata.yaml` for the recording set

Environment overrides supported by the script:

- `STORAGE_ID`
- `MAX_BAG_DURATION_SECONDS`
- `MAX_CACHE_SIZE_BYTES`

Example:

```bash
MAX_BAG_DURATION_SECONDS=60 STORAGE_ID=sqlite3 \
  ./project_launchers/record_rx2_urban_eval_v1.sh
```

## Typical Workflows

### Live sensors plus recording

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch project_launchers/rx2_urban_eval_v1.xml
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./project_launchers/record_rx2_urban_eval_v1.sh
```

### Offline replay from packet bags

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch project_launchers/rx2_urban_eval_v1.xml online_sensor:=false
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag play /path/to/packet_bag
```

Terminal 3, if you also want a derived pointcloud bag:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./project_launchers/record_rx2_urban_eval_v1.sh /data/bags/rx2_urban_eval_v1_decoded
```

## Notes

- The launcher is project-local, not package-installed, so it should be launched by relative path from the repo root.
- The full offline dry test has already been run successfully with `online_sensor:=false`.
- If Hesai decoder behavior changes, rebuild `nebula_hesai_decoders` and `nebula_hesai` before retesting.
