# LiDAR perimeter alignment

This node corrects `map -> odom` using fixed, tall perimeter planes in the current
OriginSim arena. It consumes `/cloud_registered` and `/odom`; it never consumes
Gazebo truth. The operator-provided initial X/Y/Z/Rotation calibration remains
required. `/map -> odom` is withheld until eight stationary scans establish at least three
supported axis-aligned perimeter planes with both normal directions represented. Do not run `modify_map_to_odom` at the
same time.

Points above `landmark_min_height` (default 4 m in this simulated arena) exclude
internal obstacles and ramps. Subsequent robust plane fitting updates the frame
in bounded increments. Sparse, unsupported, high-residual or implausibly large
updates are rejected. This is an arena-specific landmark localizer, not a
replacement for general 3D localization in arbitrary maps. It requires visible
tall planar perimeter structures and a calibrated start; outside these conditions
use an appropriately configured global localizer.

Point-LIO's optional `odometry.predict_with_imu` supplies bounded, high-rate
constant-velocity prediction and gyro attitude propagation between LiDAR updates.
It requires continuous fresh IMU and a LiDAR correction no older than 1 second.
The navigation executor retains its 0.5 second input timeout. Raw acceleration
is deliberately not integrated for the short output prediction because gravity
and bias errors on ramps can dominate the position estimate. The output velocity prior is estimated from corrected LiDAR positions over a 0.5 second window, to avoid retaining filter velocity bias when stopped. The LiDAR filter itself is unchanged by these output predictions.

## Wheel / IMU / raw LiDAR backend

`wheel_lidar_odometry.py` is the separate backend selected by
`bringup/no_smoother_tunnels.launch.py`. Do not run it together with Point-LIO,
`landmark_alignment.py`, or another publisher of `/odom` / `map -> odom`.
It consumes only `/joint_states`, `/livox/imu`, and the raw
`/livox/lidar/pointcloud`; simulator `/Odometry` is used solely by external tests.

Initialize stationary on a flat surface with the chassis X axis along map X and
calibrate the initial X/Y/height. Sensor mounting yaw is -1.095 rad, the IMU
lever arm is 0.6 m, and the LiDAR offset above it is 0.03 m. IMU attitude is
conjugated by the mounting rotation. Wheel rolling uses the horizontal wheel
axle projection, which remains valid when pitch crosses 90 degrees. Planar Z is
fixed; this backend does not estimate heave.

Eight stationary scans learn at least three fixed tall perimeter planes spanning
both axes. Regular corrections require supported long wall segments in both
axes. Following encoder slip, reacquisition additionally requires at least three
separately matched planes, both axes, agreement within 0.06 m per axis, and a
translation innovation no greater than 1 m. Position correction is limited to
0.15 m per cloud; large innovations suppress odometry until alignment recovers.
A 0.4 s window of corrected scan positions supplies observed velocity, so a
spinning tire is not treated as actual progress.

Wheel and IMU inputs must be within 0.1 s of ROS time and 0.3 s of receipt time.
Wall constraints must be within 0.5 s of ROS time and 0.8 s of receipt time.
Otherwise odometry output stops and the executor's input watchdog stops commands.
The profile requires visible tall axis-aligned perimeter walls and calibrated
entrance starts. It is specific to this static arena; it does not establish
long-range or arbitrary-map localization performance.
