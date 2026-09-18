# Algorithm profiles

`simulation.sh` starts Gazebo and publishes sensor/actuator interfaces; it does
not start a navigation algorithm. `algorithm_sim.launch.py profile:=NAME` loads
`NAME.yaml` separately. Keep algorithm-specific frames, parameters and topic
remappings in the profile or its adapter, without changing algorithm sources.

Any node or included launch action can optionally wait for sensor readiness:

```yaml
actions:
  - type: node
    package: your_algorithm
    executable: your_node
    name: your_algorithm
    wait_for_sensors:
      base_frame: base_link
      lidar_frame: livox_frame
      imu_topic: /livox/imu
      require_static_imu: true
      acceleration_norm: 9.81
      acceleration_tolerance: 1.0
      sample_count: 200
      stable_duration: 0.9
      max_angular_speed: 0.1
      max_acceleration_stddev: 0.15
```

The gate checks TF and sensor data, then exits successfully; only then does
launch start the configured action. It does not compute or inject gravity into
the algorithm. `require_static_imu: false` checks only TF; omit
`wait_for_sensors` to start the action without a sensor gate. Units must match
the published IMU data, and thresholds must match the algorithm's requirements.

For profiles using the navigation lifecycle manager, `navigation_tf_gate`
optionally specifies the `target_frame` and `source_frame` required before
activation. Profiles that omit it do not use that additional TF gate.

The navigationros2 profile launches Small Point-LIO, NDT localization,
segmentation, sentry decision, Nav2 map/planner/controller/BT/behaviors and
velocity adapters. Gazebo replaces physical lidar/serial drivers and provides
the robot description and joint feedback. The initializer is the sole
map-to-odom owner; the initial-pose adapter only converts base_links poses to
physical base_link poses and enables navigation after NDT reports success.

Map selection belongs to the algorithm: NDT reads map_file directly from
localization_initializer/config/initializer_params.yaml; map_server reads the
map argument default from mybringup/launch/run_nav.launch.py. The profile does
not specify alternate map files. Launch validates the PCD, map YAML and image
before starting nodes instead of permitting a missing-map fallback.
No additional rotation is applied to the PCD by the simulation adapter.

The sentry decision node retains its real algorithm inputs for game, friendly
robot and enemy state. Gazebo motion simulation does not synthesize these;
automatic match decisions require an external publisher supplying them.

In wheel simulation, a detachable fixed physics joint connects chassis to an
invisible world-fixed anchor at spawn. Only a finite, nonzero planar command
selected by cmd_vel_mux on /cmd_vel_chassis releases it. Zero commands, initial
poses, localization success and gimbal commands do not release the chassis.
After release, command timeouts stop the drive normally without reattaching.
The /simulation/robot_released Bool reports whether detachment has completed.
Physical ground contact is still required after detachment.
# navigationros2 ground odometry

The simulation profile overrides IMU acceleration units consistently:
acc_norm=9.81 and satu_acc=34.335 (3.5 g in m/s^2). Both simulated sensors
are at the livox_frame origin, so their relative translation is zero.
supervised_lio.py relays the actual base_link->livox_frame TF onto an
isolated static topic and repeats it for late discovery. It restarts LIO
if the algorithm reports its identity-extrinsic fallback. The public
ground adapter suppresses outputs until successful extrinsic caching is
confirmed; algorithm source files and the physical sensor mounting are unchanged.

The navigationros2 profile isolates Small Point-LIO's dynamic TF, odometry,
and registered cloud on /lio/raw_tf, /lio/raw_odometry, and
/lio/raw_cloud_registered. lio_ground_adapter.py freezes one translation
from the first valid base_link pose and its base_links mounting transform.
It applies that same translation to all odom-parent TF edges, odometry
positions, and cloud XYZ. Sensor extrinsics, timestamps, orientation,
child-frame velocity, intensity, and covariances retain their original values.
The initial base_links position is the public odom origin; no continuous
height clamp is applied. NDT remains the only map->odom publisher.
Before localization its identity transform places both odom and the initial
ground reference on map z=0. Later 3D localization can still estimate a
nonzero map->odom height. Raw LIO /map_save output is not converted by this
adapter and must not be assumed to use the public ground odom coordinates.
