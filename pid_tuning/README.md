# Gazebo PID tuning

All tools/results here are independent of production source and launch configuration.
Actuation requires `ROS_DOMAIN_ID=77 GZ_PARTITION=seu_pid_tuning`.
Source ROS Jazzy, navigationsim/install/setup.bash and navigationros2/install/setup.bash first.

`initialize.py` sends one initial pose: ground base_links at map (0,0,0), gimbal base_link +X along map +X. Do not repeatedly reset a rotating robot.

`nav2_trial.py --live-localization --goal X Y --navigate --gains WP WI WD SP SI SD --output results/name.json` uses real Nav2 ComputePathToPose and NavigateToPose. Without --navigate it sends the original computed plan to the real FollowPath controller. All errors use the immutable original plan and Gazebo truth, transformed once to map at the start. Subsequent replans do not replace the scoring reference. No teleportation in full LIO trials. Gains are transient requests through the existing Gazebo interface; no readback ACK exists. Source gains are restored after each trial. The --speed option is reserved and does not override navigation speed; actual production desired speed is currently 2.3 m/s.

`nav2_plot.py results/name.json` plots path, full trajectory, cross-track error, command/actual speed and chassis angular speed. Trials record wheel and steer joints, all three command-conversion stages, and commanded motor torques via the independent read-only observer. Torque observation is not a contact-force measurement.

A pass requires action success, sampled maximum cross-track <=0.1 m, stopped final position error <=0.1 m, no missing TF and time-aligned localization disagreement <=0.1 m. Recording is at approximately 50 Hz; this cannot certify unsampled continuous-time extrema. Three seconds after action completion are recorded to include stopping. Navigation's own goal tolerance is not the acceptance threshold.

`trial.py`, `basic.py`, `matrix.py`, `tune.py` are lower-level plant tests, not final Nav2 validation. Their direct chassis commands bypass virtual-heading conversion and must not be reported as navigation success. `truth_fixture.py` and `nav2_test_profile.yaml` are isolated diagnostic fixtures and must not run beside full LIO.

`ask_model.py` uses measured data and the local private `.llm_config.json`. The supplied relay's working transport is streamed Responses. The upstream llm-pid-tuner is unchanged. Model proposals must pass real Gazebo tests before adoption.

Current full-algorithm results have NOT met 0.1 m. Do not label a successful Nav2 action or good terminal arrival as a successful full-path trial.
