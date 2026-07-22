# mc_state_observation

This package implements additional state observers for `mc_rtc`.
Some will be considered for inclusion in `mc_rtc` once they have been fully battle tested.

Here is an overview of the various observers implemented:

## Observers

### AttitudeObserver (estimation of IMU orientation)

This observer is directly inspired by the [AttitudeEstimator](https://github.com/isri-aist/hrpsys-state-observation/blob/master/include/hrpsys-state-observation/AttitudeEstimator.h) of [hrpsys-state-observation](https://github.com/isri-aist/hrpsys-state-observation) (that provides an improved replacement for the deprecated [`KalmanFilter`](https://github.com/isri-aist/hrpsys-private/tree/master/KalmanFilter) component of [hrpsys-private](https://github.com/isri-aist/hrpsys-private)). The `AttitudeEstimator` component has been heavily used on `HRP-5P`.

Configuration options:

```yaml
robot: JVRC1                # robot to observe (defaults to the main robot)
imuSensor: Accelerometer    # sensor providing the IMU measurements (defaults to the first bodysensor)
updateSensor: Accelerometer # name of the sensor in which to write the estimated orientation (defaults to imuSensor)
log_kf: false               # whether to log the kalman filter parameters (default: false)
init_from_control: true     # whether to initialize the kalman filter's orientation from the control robot state (default: true)
KamanFilter:                # configuration of the kalman filter (default values should be reasonable in most cases)
  compensateMode: true
  offset: [0,0,0]           # Apply an orientation offset to the estimation result (rpy or matrix)
  acc_cov: 0.003
  gyr_cov: 1e-10
  ori_acc_cov: 0.003
  lin_acc_cov: 1e-13
  state_cov: 3e-14
  state_init_cov: 1e-8
```

### MocapObserverROS (estimation of the floating base from MOCAP data)

Example configuration (updates main real robot instance from MOCAP data). Note that this requires calibration of the mocap marker wrt to the robot body:
- `Calibrate`: Measures the marker frame to robot body transformation (calibration of the MOCAP markers). This assumes that the initial robot position is very well known in the controller
- `Initialize`: Establishes the link between robot map and mocap origin.

```
ObserverPipelines:
- name: MocapPipeline
  gui: true
  observers:
    - type: Encoder
    - type: MocapObserverROS
      update: true
      config:
        updateRobot: hrp5_p
        marker_tf: HRP5P
        marker_origin_tf: mocap
        body: Chest_Link2
```

### SLAMObserver (Experimental)

Estimation of the robot thanks to the estimated camera from a SLAM.

Configuration options (default value for `Filter`, `Publish` and `Simulation`):

```yaml
Robot:
  robot: robot                      # Robot name (Optionnal, by default it will be the main robot name)
  robot_name_0:                     # Must be a name of a valid robot
    camera: camera_link             # Body name of the camera of the robot name 0
  robot_name_n:                     # Must be a name of a valid robot
    camera: camera_link             # Body name of the camera of the robot name 1
  ground: ground                    # Ground frame to have a ground pose in SLAM map
SLAM:
  map: map                          # ROS TF name of SLAM map
  estimated: camera_link            # ROS TF name of estimated camera
Filter:
  use: true
  m: 100                            # savitzky-golay parameters
  d: 2                              # savitzky-golay parameters
Publish:
  use: true                         # publish estimated robot in ROS
GUI:
  plots: true                       # Enable SLAM plots in mc_rtc GUI (can be disabled/enabled at runtime)
Simulation:
  use: false                        # If true, set estimated to rea/camera of robot
  noise:
    use: false
    translation:
      min: [-0.05, -0.05, -0.05]    # [m]
      max: [0.05, 0.05, 0.05]       # [m]
    orientation:
      min: [-0.01, -0.01, -0.01]    # [degree]
      max: [0.01, 0.01, 0.01]       # [degree]
```

In your controller's configuration file in .yaml:
```yaml
ObserverPipelines:
- name: SLAMPipeline
  gui: true
  observers:
    - type: Encoder
    - type: SLAM
      update: true
      config:
        Robot:
          robot: hrp2_drc
          hrp2_drc:
            camera: xtion_link
        SLAM:
          map: map
          estimated: camera_link
        Simulation:
          use: false
```

Then from your controller you can access to the estimated robot with:
```cpp
const auto & estimatedRobot = datastore().call<const mc_rbdyn::Robot &>("SLAM::Robot");

bool isAlive = datastore().call<bool>("SLAM::isAlive");
```

### ObjectObserver (Experimental)

Estimation of the object thanks to the estimated object from a vision process.
Configuration options (default value for `Filter`, `Publish` and `Simulation`):

```yaml
Robot:
  robot: robot                      # Robot name (Optionnal, by default it will be the main robot name)
  robot_name_0:                     # Must be a name of a valid robot
    camera: camera_link             # Body name of the camera of the robot name 0
  robot_name_n:                     # Must be a name of a valid robot
    camera: camera_link             # Body name of the camera of the robot name 1
Object:
  robot: object                     # Robot name
  topic: /topic/poseStamped         # ROS topic to receive estimated object pose stamped
  inRobotMap: false                 # If the update is compute from robot camera or from robot_map (in case of choreonoid by example)
Publish:
  use: true                         # publish estimated robot in ROS
```

In your controller's configuration file in .yaml:
```yaml
ObserverPipelines:
- name: ObjectPipeline
  gui: true
  observers:
    - type: Object
      update: true
      config:
        Robot:
          robot: hrp2_drc
          hrp2_drc:
            camera: xtion_link
        Object:
          robot: cube
          topic: /process/cube/pose
```

Then from your controller you can access to the estimated robot with:
```cpp
const auto & estimatedRobot = datastore().call<const mc_rbdyn::Robot &>(name_+"::Robot");

const auto & estimatedRobot = realRobot(name_);

const auto & X_0_Object = datastore().call<const sva::PTransformd &>(name_+"::X_0_Object");

const auto & X_0_Object = realRobot(name_).posW();

const auto & X_Camera_Object = datastore().call<const sva::PTransformd &>(name_+"::X_Camera_Object");
```

## Dependencies

- [gram_savitzky_golay](https://github.com/arntanguy/gram_savitzky_golay)
- [state-observation](https://github.com/jrl-umi3218/state-observation) > 1.3.3
- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc)
- Eigen3
- Boost

## Install from APT

```bash
# For head version replace stable with head
curl -1sLf 'https://dl.cloudsmith.io/public/mc-rtc/stable/setup.deb.sh' | sudo -E bash
# For the Attitude observer
sudo apt install mc-state-observation
# For ROS-based observers
sudo apt install ros-${ROS_DISTRO}-mc-state-observation
```


## Summary of the Observers

- **Tilt Observer**: [*Lyapunov-Stable Orientation Estimator for Humanoid Robots*](https://ieeexplore.ieee.org/document/9158355), by Benallegue et al., IEEE Robotics and Automation Letters, 2020. \
Adaptation of the Tilt Observer to the case of humanoid robots, where the IMU's local linear velocity measurement is obtained using the assumption of fixed contacts.
    - Measurements:
        - IMU's accelerometer and gyrometer.
        - Robot's joint encoders.
    - Inputs:
        - Center of pressure of the contacts.
    - State:
        - IMU's local linear velocity.
        - IMU's tilt ($=\boldsymbol{R}^{T}\boldsymbol{e}_{z}$): the inclination of the IMU with respect to the vertical axis.
    - Features:
        - Based on a complementary filter, allowing for a very fast computation.
        - The estimation error's dynamics is autonomous and its convergence is mathematically proven.


- **Kinetics Observer**: [*The Kinetics Observer: A Tightly Coupled Estimator for Legged Robots*](https://hal.science/hal-04616647), by Demont et al.
    - Measurements:
        - IMU's accelerometer and gyrometer.
        - Robot's joint encoders.
        - Force / torque sensors.
    - State:
        - Centroid frame's position ($\boldsymbol{p}_{l}=\boldsymbol{R}^{T}\boldsymbol{p}$) and orientation in the world.
        - Centroid frame's linear ($\boldsymbol{v}_{l}=\boldsymbol{R}^{T}\dot{\boldsymbol{p}}$) and angular velocities in the world.
        - Gyrometer measurement biases.
        - Resultant of the unmodeled external wrench applied to the robot, expressed in the centroid frame.
        - Contact rest poses in the world.
        - Contact reaction wrench.
    - Features:
        - Based on a Multiplicative Extended Kalman Filter.
        - Performs a proprioceptive odometry, which is corrected by the wrench sensor measurements. This provides the observability of contact slippage in the frame attached to the robot.


## Configuation of the observers within mc_rtc

Here is a list and explanation of the main parameters for the configuration of the observers in mc_rtc. An example of configuration can be found in [this repository](https://github.com/ArnaudDmt/mc_rtc_Configs), which can be locally located in ~/.config. The configuration of an observer can be customized for each robot.

### ContactsManager

Some observers require a detection of contacts, which is done by the ContactsManager. It can be configured in the observer's configuration file with the synthax:
```
contacts:
  contactsDetection: Surfaces / Sensors / Solver
  schmittTriggerLowerPropThreshold # (default: 0.10)
  schmittTriggerUpperPropThreshold # (default: 0.15)
  # surfacesForContactDetection: [RightFootCenter, LeftFootCenter, LeftHandCloseContact] # List of surfaces to consider when using 'Surfaces' as the detection method.
```
Contacts can be detected with three different methods.
- `Surfaces`: the contact manager will focus on a given list of surfaces the robot can have with the environment. This list is added to the configuration as surfacesForContactDetection. A contact is detected if the force measured at the sensor attached to the surface is above the lower threshold of the Schmitt Trigger. Its name is the force sensor's name and its frame is the surface's frame.
- `Sensors`: the contact manager will consider all the robot's force sensors. A contact is detected if the force measured at the sensor attached to the surface is above the lower threshold of the Schmitt Trigger. Its name is the force sensor's name and its frame is the force sensor's frame (and not of the surface in contact!).
- `Solver`: the list of contacts is given by the controller, and accessed by the contacts manager through the command 'ctl.solver().contacts()'.

### Legged Odometry

Some observers are able to perform odometry, **when not used for the control feedback**. The odometry mode can be configured with the synthax:
```
leggedOdometry:
  odometryType: None / 6D / Flat
  correctContacts: true # defines if the contact reference poses must be corrected over time
```
Possible modes:
- `None`: (default) the observer remains attached to the control robot and can be used as a feedback for the controller.
- `6D`: the observer performs 6D odometry. Its pose is obtained using the sensor measurements and is independent from the one of the control robot.
- `Flat`: odometry for walks on flat ground. Contacts are created with a zero height, constraining the robot to the ground. Use the 6D odometry if you intend to create other kinds of contacts.


### Tilt Observer

The Tilt Observer is able to perform [odometry](#legged-odometry).
```
leggedOdometry:
  # See above

filterGains:
  initAlpha: 5 # (default: 5)
  initBeta: 1 # (default: 1)
  initGamma: 2 # (default: 2)

  finalAlpha: 5 # (default: 5)
  finalBeta: 1 # (default: 1)
  finalGamma: 2 # (default: 2)
```
Parameters:
  - `alpha`: gain $\alpha_{1}$ in [the paper](https://ieeexplore.ieee.org/document/9158355), related to the convergence of the linear velocity $\hat{\boldsymbol{x}}_{1}$.
  - `beta`: gain $\alpha_{2}$ in [the paper](https://ieeexplore.ieee.org/document/9158355), related to the convergence of the intermediate tilt $\hat{\boldsymbol{x}}_{2}^{\prime}$.
  - `gamma`: gain $\gamma$ in [the paper](https://ieeexplore.ieee.org/document/9158355), related to the convergence of the tilt $\hat{\boldsymbol{x}}_{2}$.

### Kinetics Observer

The Kinetics Observer is able to perform [odometry](#legged-odometry). To configure it, please see the [dedicated section](#legged-odometry).
```
leggedOdometry:
  # See above
```
As in [this repository](https://github.com/ArnaudDmt/mc_rtc_Configs/tree/main/observers), the Kinetics Observer can be configured with **global** and **robot-specific** parameters.

#### Parameters:

  - The contact linear and angular flexibilities: `linStiffness`, `angStiffness`, `linDamping`, and `angDamping`. Often robot-specific.
  - The Kalman Filter covariances:
    ```
    ekfStateProcessVariances:
      ... # initial state covariances and process covariances
    ekfSensorNoiseVariances:
      # measurement covariances. Often robot-specific.
    ```
  - `backupInterval`: capacity (in seconds) of the buffer that allows to recover the state if the estimation diverges. The Tilt Observer is ran internally and used for the recovery.
  - `withDebugLogs`: add all the debugging logs.
  - `withGyroBias`: defines whether the gyrometer bias must be estimated.
  - `withUnmodeledWrench`: defines whether the unmodeled external wrench must be estimated.
