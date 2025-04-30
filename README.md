# Task Manager for Terrawarden MQP
## Overview


## Run Instructions:

`ros2 launch task_manager just_manager.launch.py`
`ros2 launch task_manager all_nodes.launch.py`

Move into SEARCH from HOLD with `ros2 topic pub -1 /manager_set_state std_msgs/msg/String "{data: 'SEARCH'}"`

Recommended Topic Listeners:
- `ros2 topic echo /drone/telemetry`
- `ros2 topic echo /manager_state`

## State Machine:

![](media/state_diagram.png)

The task manager has ten states. Of these, six are behavior-states, two are for transitions, and another two are for failsafe conditions 
```
class State(Enum):
    STARTUP = "STARTUP"     # Does nothing until drone is moved into offboard mode
    HOLD = "HOLD"           # Holds position until commanded to move into a different state
    SEARCHING = "SEARCH"    # Slowly spins until target object detected
    NAVIGATING = "NAV"      # Approaches object until stopped at a fixed offset from object (with manipulator workspace) 
    TRACKING = "TRACK"      # Live track objects position using manipulator, with an offset to not block camera view of object
    GRASPING = "GRASP"      # Once aligned with object, attempts to grasp can. Checks gripper current to verify
    DEPOSITING = "DEPOSIT"  # Flies back to home position with object and releases
    WAITING = "WAIT"        # Adds a time delay or boolean condition between state transitions
    FAILSAFE = "FAILSAFE"   # Flies back to home position in condition of RC signal lost
    LANDING = "LAND"        # After arriving back at home position in FAILSAFE, attempts to land autonomously
```


## Running Unit Tests
From ROS2 workspace root:
`colcon test`
`colcon test-results --all`

## Launch Arguments

### Debug & General Settings

| Name         | Default | Description |
|--------------|---------|-------------|
| `manager_debug`|`0x11111`| Bitwise debug flags to control which parts of code can log messages. Bits in order (MSB first): publish, drone, detect, vbm, arm |
| `override_errors`|`False`| Disables failsafe and ground clearance error checks for bench testing |
| `enable_stow`|`True`| Disables failsafe and ground clearance error checks for bench testing |

### Topic Configuration

| Name                      | Default              | Type                          | Description                                |
|---------------------------|----------------------|-------------------------------|--------------------------------------------|
| `drone_pose_topic`        | `/drone/waypoint`    | `CustomMessages/DroneWaypoint`  | Sends target pose to drone                 |
| `drone_telemetry_topic`   | `/drone/telemetry`   | `CustomMessages/DroneTelemetry` | Receives Telemetry from Drone Node         |
| `detection_topic`         | `/live_detection`    | `vision_msgs/Detection2D`       | Receives Detection from LiveDetect Node    |
| `vbm_extract_topic`       | `/extract_centroid`  | `geometry_msgs/PointStamped`    | Receives 3D point from VBM extract_cluster |
| `vbm_grasp_topic`         | `/grasp_read`        | `geometry_msgs/PoseStamped`     | Receives grasp from VBM optimal_grasp      |
| `grasp_command_topic`     | `/force_grasp`       | `std_msgs/Bool`                 | Sends gripper open/close commands          |
| `arm_command_topic`       | `/move_arm_command`  | `CustomMessages/ArmCommand`     | Sends trajectories to Arm Node             |
| `arm_status_topic`        | `/arm_status`        | `CustomMessages/ArmStatus`      | Status message from Arm Node               |
| `arm_stow_service_topic`  | `/stow_arm`          | None                          | Stow Service Call to Arm Node              |
| `arm_unstow_service_topic`| `/unstow_arm`        | None                          | Unstow Service Call to Arm Node            |
| `state_topic`             | `/manager_state`     | `std_msgs/String`               | Sends current manager state information    |
| `state_setter_topic`      | `/manager_set_state` | `std_msgs/String`               | Recieves set_state information             |

### all_nodes.launch.py

This launch file launches the TaskManager alongside all other nodes necessary for the operation of this drone, including `ManipulatorControl`, `DroneControl`, `LiveDetection`, `VBM_ExtractCluster`, and the Intel Realsense nodes. It contains all launch arguments for this node as well as the aforementioned (except realsense, which has been configured with tuned values), with no changes to argument names.

## TODO
Some ideas future years. These are not comprehsensive and not in any particular order. 
1) Better search procedures
2) An actual implementation for DEPOSIT (original thought was to search for a trash can lol)
3) Removal of some unused code?
4) Merge in rest of unit tests from unit_tests branch, write tests for all nodes
5) Merge in code standardization steps from the standardizing-code branch
