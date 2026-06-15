# ROS2 Integration Plan: `rtk_engine_ros`

## Objective
To provide a drop-in ROS2 node that wraps the `RtkEngineApp` for seamless use in autonomous navigation stacks.

## Architecture
- **Node:** `rtk_node`
- **Subscribed Topics:**
  - `/gnss/raw_obs` (Custom type or `sensor_msgs/NavSatFix`)
  - `/imu/data` (`sensor_msgs/Imu`)
- **Published Topics:**
  - `/gnss/fix` (`sensor_msgs/NavSatFix`)
  - `/gnss/velocity` (`geometry_msgs/TwistStamped`)
  - `/gnss/rtk_status` (Custom status)

## Implementation Steps
1.  Define custom message types for raw observation streaming.
2.  Create the `rtk_node` class inheriting from `rclcpp::Node`.
3.  Implement callbacks to map ROS messages to internal `rtk::EpochObs` and `rtk::ImuMeas` types.
4.  Configure `package.xml` and `CMakeLists.txt` for ROS2 workspace build.
