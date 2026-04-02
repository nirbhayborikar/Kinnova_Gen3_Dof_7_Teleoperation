# Robotiq 2F-140 Gripper Parameter Modifications

## Overview
This document outlines the parameter changes made to adapt the ros2_robotiq_gripper repository for the Robotiq 2F-140 gripper. The original repository was optimized for the 85mm gripper, requiring modifications to ensure proper functionality with the 140mm variant.

## Problem Statement
The original configuration had several issues when used with the 2F-140 gripper:

1. **Closing Position Mismatch**: The default close value of `0.7` would fully close the URDF model but not the physical gripper
2. **URDF vs Physical Gripper Discrepancies**: Significant differences between the simulated and real gripper behavior
3. **Joint Limit Inconsistencies**: The original joint limits were not optimized for the 140mm stroke

## Solution Overview
The primary fix involved adjusting the mimic joint multiplier for the right side of the gripper from `-1.0` to `-0.875`, along with corresponding joint limit modifications.

## Modified Parameters

### 1. ROS2 Control Configuration
```xml
<!-- Increased maximum closing position for better real gripper representation -->
gripper_closed_position:=0.7929  <!-- Previously: 0.695 -->
```

### 2. Joint Limits 
```xml
<!-- Main finger joint - increased upper limit -->
<limit lower="0" upper="0.8" velocity="2.0" effort="1000" />
<!-- Previously: upper="0.7" -->

<!-- Right outer knuckle joint - increased limits -->
<limit lower="-0.825" upper="0.825" velocity="2.0" effort="1000" />
<!-- Previously: lower="-0.725" upper="0.725" -->
```

### 3. Mimic Joint Multipliers
```xml
<!-- Right outer knuckle joint mimic -->
<mimic joint="${prefix}finger_joint" multiplier="-0.875" offset="0" />
<!-- Previously: multiplier="-1" -->

<!-- Right inner knuckle joint mimic -->
<mimic joint="${prefix}finger_joint" multiplier="-0.875" offset="0" />
<!-- Previously: multiplier="-1" -->

<!-- Right inner finger joint mimic -->
<mimic joint="${prefix}finger_joint" multiplier="0.875" offset="0" />
<!-- Previously: multiplier="1" -->
```

## Technical Rationale

### Why 0.875 Multiplier?
The `0.875` multiplier was chosen to:
- Provide a more accurate representation of the physical gripper's closing behavior
- Maintain proper synchronization between left and right finger movements
- Prevent the URDF from appearing "broken" when using the increased `0.8` upper limit

### Joint Limit Adjustments
The joint limits were expanded to:
- Accommodate the larger stroke of the 140mm gripper
- Provide better range of motion matching the physical hardware
- Ensure proper operation across the full gripper range

## Implementation Notes

1. **Conditional Logic**: The mimic multipliers use conditional logic to apply different values for left vs right fingers
2. **Backward Compatibility**: Changes are isolated to the 140mm gripper configuration
3. **Safety Margins**: Joint limits include appropriate safety margins for real-world operation

## Testing Recommendations

When implementing these changes:
1. Verify URDF visualization matches expected gripper behavior
2. Test full range of motion in simulation
3. Validate closing position accuracy with physical hardware
4. Confirm proper mimic joint behavior during operation

## Files Modified
- `robotiq_gripper_ros2_control.xacro` - ROS2 control parameters
- `robotiq_2f_140_gripper_joint_limits.xacro` - Joint definitions and limits

These modifications ensure proper functionality of the Robotiq 2F-140 gripper with the ros2_robotiq_gripper package while maintaining realistic simulation behavior.