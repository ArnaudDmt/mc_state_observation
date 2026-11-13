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

This package provides estimators for the state of legged robots, built on the [state-observation](https://github.com/jrl-umi3218/state-observation) library.

Until 2026, the standard observation pipeline was using the combination of Attitude Observer and Kinematic Inertial. Attitude Observer estimates the robot's tilt, and Kinematic Inertial uses this tilt to compute the floating base pose and the anchor point defined by the control robot's feet. This provides a feedback for the robot's stabilization that remains attached to the control robot.

From 2026, we added the Tilt Observer, Valinor, WAIKO, and the Kinetics Observer, which are meant to replace this combination.
In summary, Tilt, Valinor and WAIKO share the same complementary-filter core, providing a fast estimation of the robot kinematics whose error has global convergence guarantees. The Kinetics Observer uses a Kalman filter, and additionally estimates the contact state rest pose, contact forces, and the perturbation wrench applied to the robot's centroid. Its odometry is also improved by the use of contact wrench measurements.

In more details:

### Tilt Observer

[*Lyapunov-Stable Orientation Estimator for Humanoid Robots*](https://ieeexplore.ieee.org/document/9158355), by Benallegue et al., IEEE Robotics and Automation Letters, 2020.

Adaptation of the Tilt Observer to the case of humanoid robots, where the IMU's local linear velocity measurement is obtained using the assumption of fixed contacts.
- Measurements of the filter:
    - IMU's accelerometer $\boldsymbol{y}_{a}$ and gyrometer $\boldsymbol{y}_{g}$.
    - IMU's local linear velocity $\boldsymbol{y}_{v}=-\boldsymbol{y}_{g}\times\boldsymbol{p}_{a}-\dot{\boldsymbol{p}}_{a}$, obtained from the position $\boldsymbol{p}_{a}$ and velocity $\dot{\boldsymbol{p}}_{a}$ of the anchor frame in the IMU frame (assumed to have zero velocity in the world). They are computed with the joint encoders, from the feet of the robot, or from a frame given by the controller (see [`anchorFrameFunction`](#configuration-of-the-tilt-observer)).
- State:
    - IMU's local linear velocity $\boldsymbol{x}_{1}=\boldsymbol{R}^{T}\dot{\boldsymbol{p}}$, $\boldsymbol{R}$ and $\boldsymbol{p}$ being the orientation and position of the IMU in the world.
    - Intermediate estimate of the tilt $\boldsymbol{x}_{2}'$, used to obtain a fast convergence.
    - IMU's tilt $\boldsymbol{x}_{2}=\boldsymbol{R}^{T}\boldsymbol{e}_{z}$: the inclination of the IMU with respect to the vertical axis.
- Output: it is the closest design to `Attitude` + `KinematicInertial`. The estimated tilt is merged with the yaw of the control robot, and the position is obtained from the anchor frame of the control robot, as in `KinematicInertial`. **It does not perform odometry.**
- Features:
    - Based on a complementary filter, allowing for a very fast computation.
    - The estimation error's dynamics is autonomous and its convergence is mathematically proven.

### Valinor

[*VALINOR: A Minimalist Leg Inertial Odometry for Humanoid Robots*](https://hal.science/hal-05506999), by Demont et al., International Journal of Control, Automation and Systems, 2025.

- Extends the Tilt Observer with a legged odometry, performed in a **decoupled** way: the tilt remains estimated by the Tilt Observer, and the position and yaw are obtained from leg odometry, explained below.
- The pose of each contact is computed using forward kinematics from the current IMU kinematics. It is then considered constant, providing for all consecutive iterations a measurement of the IMU position and yaw through forward kinematics. The poses obtained from multiple contacts are averaged, with a weighting based on the measured force at the contact.
- Filter: same measurements and state as the Tilt Observer, except that the anchor frame is computed from the currently detected contacts.
- Legged odometry: takes the estimated tilt and the kinematics of the contacts in the IMU frame, and outputs the position and orientation of the IMU in the world.
- Sensors: IMU, joint encoders, and the force sensors associated to the contact surfaces (used to detect the contacts and weight them).

### WAIKO

Introduced in [this thesis](https://theses.hal.science/tel-05532948).

- Extends Valinor by performing the legged odometry **directly inside the complementary filter** (and not in a decoupled way at the end of the complementary filter). This has several advantages:
  - the feed forward on the position estimate is the velocity estimated within the filter, which is accurate and whose global convergence is proven. Leg kinematics are also used to correct the estimated pose.
  - by assuming contact reference poses are fixed in the world and thus known, we can also derive a proof of almost global convergence of the error on the pose.
- Inputs of the filter:
    - IMU's accelerometer $\boldsymbol{y}_{a}$ and gyrometer $\boldsymbol{y}_{g}$.
    - Leg kinematics, computed with the joint encoders:
        - the IMU's local linear velocity $\boldsymbol{y}_{v}$, obtained from the anchor frame as in the Tilt Observer;
        - the position ${}^{\mathcal{I}}\boldsymbol{p}_{c}$ of each contact in the IMU frame, with its weight;
        - the orientation of the IMU in the world obtained from the contact orientations.
    - The reference poses of the contacts in the world (from the legged odometry, assumed fixed).
- State:
    - IMU's local linear velocity $\boldsymbol{x}_{1}=\boldsymbol{R}^{T}\dot{\boldsymbol{p}}$, as in the Tilt Observer.
    - Intermediate estimate of the tilt $\boldsymbol{x}_{2}'$, as in the Tilt Observer.
    - IMU's orientation $\boldsymbol{R}$ in the world (whose tilt $\boldsymbol{R}^{T}\boldsymbol{e}_{z}$ converges to $\boldsymbol{x}_{2}'$ with the gain `gamma`).
    - IMU's local position $\boldsymbol{p}_{l}=\boldsymbol{R}^{T}\boldsymbol{p}$, whose dynamics is $\dot{\boldsymbol{p}}_{l}=\boldsymbol{x}_{1}-\boldsymbol{y}_{g}\times\boldsymbol{p}_{l}$.
- Corrections, using the leg kinematics and the contact reference poses:
    - the local position is corrected by the IMU's local position measured from each contact: $\hat{\boldsymbol{R}}^{T}\boldsymbol{p}_{c}-{}^{\mathcal{I}}\boldsymbol{p}_{c}$, with $\boldsymbol{p}_{c}$ the reference position of the contact in the world (weighted average over the contacts, gain `rho`);
    - the yaw is corrected by the contact orientations (gain `mu`, enabled by default with `withOriCorrectFromContactOri`), and optionally, by the contact positions (disabled by default,  enabled with `withOriCorrectFromContactPos`, gain `psi`). This requires at least two contacts at different positions.
- Sensors: IMU, joint encoders. Force sensors associated to the contact surfaces are used to detect and weight contacts within the correction, but for the latter weighting, we could also use kinematical criteria.

### Kinetics Observer

[*The Kinetics Observer: A Tightly Coupled Estimator for Legged Robots*](https://hal.science/hal-04616647v3), by Demont et al.

- Inputs, computed with the joint encoders and given in the floating base frame:
    - Position, velocity and acceleration of the CoM.
    - Inertia matrix at the CoM, angular momentum around the CoM and its derivative.
    - Kinematics of the IMUs and of the contacts.
    - Resultant wrench measured by the force sensors that are not associated to a contact.
- Measurements:
    - Accelerometer and gyrometer of each IMU.
    - Force / torque sensors at the contacts.
- State:
    - Local kinematics of the centroid frame in the world: local position $\boldsymbol{p}_{l}=\boldsymbol{R}^{T}\boldsymbol{p}$, orientation $\boldsymbol{R}$, local linear velocity $\boldsymbol{v}_{l}=\boldsymbol{R}^{T}\dot{\boldsymbol{p}}$ and local angular velocity $\boldsymbol{\omega}_{l}=\boldsymbol{R}^{T}\boldsymbol{\omega}$.
    - Gyrometer biases (one per IMU, if `withGyroBias`).
    - Resultant of the unmodeled external wrench applied to the robot, expressed in the centroid frame (if `withUnmodeledWrench`).
    - For each contact: its rest pose in the world (viscoelastic contact model) and its reaction wrench.
- Features:
    - Performs proprioceptive odometry, which is corrected by the wrench sensor measurements. This provides the observability of contact slippage in the frame attached to the robot. This odometry has been extensively validated on HRP-5P and RHPS1, and showed pretty strong accuracy on both robots (see paper).
    - Runs a Valinor observer internally, used as a backup to recover the state if the estimation diverges.

### Which observer should I use?

| Need | Observer |
|---|---|
| Lowest computation cost for tilt and velocity feedback | `Tilt` |
| Fastest tilt and velocity feedback + odometry that does not need high accuracy | `MCValinor`  |
| Fast and most accurate tilt and velocity feedback, and accurate odometry (would recommend by default) | `MCWaiko` |
| Need the perturbation wrench and / or the best proprioceptive odometry. | `MCKineticsObserver` |

By default, Valinor, WAIKO and the Kinetics Observer use `odometryType: None`: they do not perform odometry and stay attached to the control robot, similarly to the Tilt Observer and Attitude Observer + Kinematic Inertial, while still providing their accurate tilt and velocity / perturbation wrench estimates. To perform odometry, set `odometryType` to `6D` or `Flat` (see [Legged odometry](#legged-odometry)).


## Configuration of the observers within mc_rtc

### How the configuration is loaded

When an observer is added to a pipeline, mc_rtc merges the following files, each one overriding the previous ones:
1. `<install prefix>/lib/mc_observers/etc/<Type>.yaml`: default configuration installed with this package (copied from the [`etc`](etc) folder).
2. `~/.config/mc_rtc/observers/<Type>.yaml`: user configuration.
3. `<install prefix>/lib/mc_observers/<Type>/<robot module>.yaml`: robot-specific configuration installed with this package (e.g. [`etc/MCKineticsObserver/robots`](etc/MCKineticsObserver/robots)).
4. `~/.config/mc_rtc/observers/<Type>/<robot module>.yaml`: user robot-specific configuration.
5. The properties given inline in the observer pipeline.

An example of user configuration can be found in [this repository](https://github.com/ArnaudDmt/mc_rtc_Configs).

Example of an observer pipeline in the controller configuration:
```yaml
ObserverPipelines:
  - name: MainPipeline
    observers:
      - type: Encoder
      - type: MCWaiko
        update: true
        # any parameter of the observer can be given here, overriding the configuration files, e.g.:
        leggedOdometry:
          odometryType: 6D
```


The default values given below are the ones obtained with the configuration files installed by this package, with no user configuration file.

### Configuration of the Tilt Observer

Parameters and their default value:
```yaml
ObserverPipelines:
  - name: MainPipeline
    observers:
      - type: Encoder
      - type: Tilt
        update: true          # (default: true)
        withDebugLogs: false  # (default: false) add debugging logs
        # anchorFrameFunction: <datastore entry> # (default: KinematicAnchorFrame::<robot>)
        filterGains:          # the init gains are used during the first second, then the final gains
          initAlpha: 4        # (default: 4) gain alpha_1 of the paper: convergence of the linear velocity x1
          initBeta: 4         # (default: 4) gain alpha_2 of the paper: convergence of the intermediate tilt x2'
          initGamma: 1        # (default: 1) gain gamma of the paper: convergence of the tilt x2
          finalAlpha: 4       # (default: 4)
          finalBeta: 4        # (default: 4)
          finalGamma: 1       # (default: 1)
```


### Configuration of Valinor

The `contacts` and `leggedOdometry` sections are described in [Contact detection](#contact-detection) and [Legged odometry](#legged-odometry).

Parameters and their default value:
```yaml
ObserverPipelines:
  - name: MainPipeline
    observers:
      - type: Encoder
      - type: MCValinor
        update: true          # (default: true)
        withDebugLogs: false  # (default: false) add debugging logs
        filterGains:          # same meaning as for the Tilt Observer. The init gains are used during the first second, then the final gains
          initAlpha: 4        # (default: 4)
          initBeta: 4         # (default: 4)
          initGamma: 1        # (default: 1)
          finalAlpha: 4       # (default: 4)
          finalBeta: 4        # (default: 4)
          finalGamma: 1       # (default: 1)
        # contacts: see Contact detection
        # leggedOdometry: see Legged odometry
```

### Configuration of WAIKO

The `contacts` and `leggedOdometry` sections are described in [Contact detection](#contact-detection) and [Legged odometry](#legged-odometry).

Parameters and their default value:
```yaml
ObserverPipelines:
  - name: MainPipeline
    observers:
      - type: Encoder
      - type: MCWaiko
        update: true          # (default: true)
        withDebugLogs: false  # (default: false) add debugging logs
        withOriCorrectFromContactOri: true  # (default: true) correct the yaw using the orientation of the contacts
        withOriCorrectFromContactPos: false # (default: false) correct the yaw using the positions of the contacts
        filterGains:          # the init gains are used during the first second, then the final gains
          initAlpha: 4        # (default: 4) same as for the Tilt Observer
          initBeta: 4         # (default: 4) same as for the Tilt Observer
          initGamma: 1        # (default: 1) same as for the Tilt Observer
          initRho: 2          # (default: 2) correction of the position by the contact positions
          initMu: 1           # (default: 1) correction of the yaw by the contact orientations
          initPsi: 0.3        # (default: 0.3, not set in the installed file) correction of the yaw by the contact positions
          finalAlpha: 4       # (default: 4)
          finalBeta: 4        # (default: 4)
          finalGamma: 1       # (default: 1)
          finalRho: 1         # (default: 1)
          finalMu: 1          # (default: 1)
          finalPsi: 0.3       # (default: 0.3, not set in the installed file)
        # contacts: see Contact detection
        # leggedOdometry: see Legged odometry
```

### Configuration of the Kinetics Observer

The `contacts` and `leggedOdometry` sections are described in [Contact detection](#contact-detection) and [Legged odometry](#legged-odometry).

Parameters and their default value:
```yaml
ObserverPipelines:
  - name: MainPipeline
    observers:
      - type: Encoder
      - type: MCKineticsObserver
        update: true          # (default: true)
        withUnmodeledWrench: true         # (default: true) estimate the unmodeled external wrench
        withGyroBias: true                # (default: true) estimate the gyrometer bias
        withAccelerationEstimation: true  # (default: true)
        withFiniteDifferences: false      # (default: false) use finite differences for the Jacobians, otherwise the analytical ones
        # finiteDifferenceStep: <double>  # required if withFiniteDifferences is true
        backupInterval: 1                 # (default: 1) capacity (in s) of the buffer used to recover the state if the estimation diverges
        withDebugLogs: false              # (default: false) add all the debugging logs
        verbose: true                     # (default: true)
        withGui: false                    # (default: false)
        filterGains:          # gains of the backup Valinor (same meaning as for Valinor, but different defaults)
          initAlpha: 4        # (default: 4, not set in the installed file)
          initBeta: 4         # (default: 4, not set in the installed file)
          initGamma: 1        # (default: 1, not set in the installed file)
          finalAlpha: 5       # (default: 5)
          finalBeta: 1        # (default: 1)
          finalGamma: 2       # (default: 2)
        # leggedOdometry: see Legged odometry
        contacts:             # contact detection parameters: see Contact detection
          linStiffness: [...]  # robot-specific: linear stiffness of the contacts
          angStiffness: [...]  # robot-specific: angular stiffness of the contacts
          linDamping: [...]    # robot-specific: linear damping of the contacts
          angDamping: [...]    # robot-specific: angular damping of the contacts
        ekfStateProcessVariances:
          # initial and process covariances, see the etc folder
        ekfSensorNoiseVariances: # measurement covariances
```

### Legged odometry

Valinor, WAIKO and the Kinetics Observer rely on the legged odometry of [state-observation](https://github.com/jrl-umi3218/state-observation) (`LeggedOdometryManager`), configured in the `leggedOdometry` section of the observer:
```yaml
leggedOdometry:
  odometryType: None    # (default) None: no odometry, the estimate stays attached to the control robot (use it for the control feedback)
                        # 6D: 3D position and orientation, independent from the control robot
                        # Flat: like 6D, but the contacts are created at zero height
  correctContacts: true # (default) correct the contact reference poses over time, with the weight
                        # (1 - lambdaInf) * exp(-kappa * t) + lambdaInf, t being the lifetime of the contact
  kappa: 10 # (default)
  lambdaInf: 0.001 # (default)
```


### Contact detection

Valinor, WAIKO and the Kinetics Observer detect the contacts with the `ContactsDetector` of this package ([`measurements/ContactsDetector.h`](include/mc_state_observation/measurements/ContactsDetector.h)). It gives the list of the currently detected contacts to the contacts management and legged odometry libraries of [state-observation](https://github.com/jrl-umi3218/state-observation). It is configured in the `contacts` section of the observer:
```yaml
contacts:
  contactsDetection: Surfaces # Surfaces (default) / Sensors / Solver. Only read by the Kinetics Observer
  surfacesForContactDetection: [RightFootCenter, LeftFootCenter] # (default) surfaces to consider with the 'Surfaces' method
  schmittTriggerLowerPropThreshold: 0.10 # (default) proportion of the robot's weight
  schmittTriggerUpperPropThreshold: 0.12 # (default) proportion of the robot's weight
```
Contacts can be detected with three different methods.
- `Surfaces`: the detector focuses on a given list of surfaces the robot can have in contact with the environment, given in `surfacesForContactDetection`, and uses the force sensor attached to each surface. The contact is named after the surface, and its frame is the surface's frame.
- `Sensors`: the detector considers all the robot's force sensors. The contact is named after the force sensor, and its frame is the force sensor's frame (and not the one of the surface in contact!).
- `Solver`: the contacts are the ones of the controller's solver (`ctl.solver().contacts()`) between the robot and a robot without degrees of freedom (the environment). The contact is named after the robot's surface.

The detection uses a Schmitt trigger on the vertical force measured by the sensor (in the sensor's frame, without the weight of the mass attached to the sensor, given by the sensor's calibration). With `Surfaces` and `Sensors`, a contact is created when this force exceeds `schmittTriggerUpperPropThreshold` times the robot's weight. With all the methods (including `Solver`, using the force sensor attached to the surface), a contact is removed when this force goes below `schmittTriggerLowerPropThreshold` times the robot's weight.

Notes:
- Valinor and WAIKO always use the `Surfaces` method (`contactsDetection` is ignored).
