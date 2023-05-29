/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#pragma once

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Robot.h>

#include <state-observation/dynamics-estimators/kinetics-observer.hpp>

#include <mc_observers/Observer.h>

namespace mc_state_observation
{
namespace so = stateObservation;
/** Flexibility observer from:
 *
 *    "Tilt estimator for 3D non-rigid pendulum based on a tri-axial
 *    accelerometer and gyrometer". Mehdi Benallegue, Abdelaziz Benallegue,
 *    Yacine Chitour. IEEE-RAS Humanoids 2017. <hal-01499167>
 *
 */

struct ForceSignal
{
  ForceSignal(bool isExternalWrench, double filteredForceZ)
  : isExternalWrench_(isExternalWrench), filteredForceZ_(filteredForceZ)
  {
  }

  bool isExternalWrench_;
  double filteredForceZ_;
  double lambda_ = 100;
};
class MapContactsIMUs
{
  /*
  Care with the use of getNameFromNum() : the mapping remains the same but the list of contacts returned buy the
  controller might differ over time (contacts broken, etc) and the id of a contact in this list might not match with its
  rank in the controller's list. Use this function preferably only to perform tasks performed on all the contacts and
  that require their name.
  */

public:
  inline const int & getNumFromName(std::string name)
  {
    return map_.find(name)->second;
  }

  inline const std::string & getNameFromNum(int num)
  {
    return insertOrder.at(num);
  }

  inline const std::vector<std::string> & getList()
  {
    return insertOrder;
  }

  inline void insertPair(std::string name)
  {
    if(map_.find(name) != map_.end()) return;
    insertOrder.push_back(name);
    map_.insert(std::make_pair(name, num));
    num++;
  }

  inline void setMaxElements(int maxElements)
  {
    // maxElements_ = maxElements_;
  }

private:
  std::vector<std::string> insertOrder;
  std::map<std::string, int> map_;
  // int maxElements_ = 4;
  int num = 0;
};

struct NaiveLegOdometry : public mc_observers::Observer
{

  NaiveLegOdometry(const std::string & type, double dt);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  void update(mc_control::MCController & ctl) override;

protected:
  void update(mc_rbdyn::Robot & robot);

  // void updateWorldFbKineAndViceVersa(const mc_rbdyn::Robot & robot);

  void initObserverStateVector(const mc_rbdyn::Robot & robot);

  void inputAdditionalWrench(const mc_rbdyn::Robot & inputRobot, const mc_rbdyn::Robot & measRobot);

  void updateIMUs(const mc_rbdyn::Robot & measRobot, const mc_rbdyn::Robot & inputRobot);

  /*! \brief Add observer from logger
   *
   * @param category Category in which to log this observer
   */

  void plotVariablesBeforeUpdate(const mc_control::MCController & ctl, mc_rtc::Logger & logger);

  void plotVariablesAfterUpdate(const mc_control::MCController & ctl, mc_rtc::Logger & logger);

  void addContactLogEntries(mc_rtc::Logger & logger, const int & numContact);

  void removeContactLogEntries(mc_rtc::Logger & logger, const int & numContact);

  void addToLogger(const mc_control::MCController &, mc_rtc::Logger &, const std::string & category) override;

  /*! \brief Remove observer from logger
   *
   * @param category Category in which this observer entries are logged
   */
  void removeFromLogger(mc_rtc::Logger &, const std::string & category) override;

  /*! \brief Add observer information the GUI.
   *
   * @param category Category in which to add this observer
   */
  void addToGUI(const mc_control::MCController &,
                mc_rtc::gui::StateBuilder &,
                const std::vector<std::string> & /* category */) override;

protected:
  /**
   * Find established contacts between the observed robot and the fixed robots
   *
   * \param ctl Controller that defines the contacts
   * \return Name of surfaces in contact with the environment
   */
  std::set<std::string> findContacts(const mc_control::MCController & solver);

  void updateContacts(const mc_rbdyn::Robot & robot,
                      const mc_rbdyn::Robot & realRobot,
                      const mc_rbdyn::Robot & inputRobot,
                      std::set<std::string> contacts,
                      mc_rtc::Logger & logger);

  void setNewContact(const std::vector<std::string> & alreadySetContacts,
                     const mc_rbdyn::Robot & robot,
                     const mc_rbdyn::Robot & realRobot,
                     const mc_rbdyn::Robot & inputRobot,
                     const mc_rbdyn::ForceSensor forceSensor,
                     mc_rtc::Logger & logger);

  void updateContact(const mc_rbdyn::Robot & robot,
                     const mc_rbdyn::Robot & inputRobot,
                     const mc_rbdyn::ForceSensor forceSensor);

protected:
  std::string robot_ = "";
  // std::string imuSensor_ = "";
  mc_rbdyn::BodySensorVector IMUs_; ///< list of IMUs

public:
  /** Get robot mass.
   *
   */
  inline double mass() const
  {
    return mass_;
  }

  /** Set robot mass.
   *
   * \param mass Robot mass.
   *
   */
  void mass(double mass);

  /** Set stiffness of the robot-environment flexibility.
   *
   * \param stiffness Flexibility stiffness.
   *
   */
  void flexStiffness(const sva::MotionVecd & stiffness);

  /** Set damping of the robot-environment flexibility.
   *
   * \param damping Flexibility damping.
   *
   */
  void flexDamping(const sva::MotionVecd & damping);

  /** Update measurement-noise covariance matrix.
   *
   */
  void updateNoiseCovariance();

  /** Get accelerometer measurement noise covariance.
   *
   */
  inline double accelNoiseCovariance() const
  {
    return acceleroSensorCovariance_(0, 0);
  }

  /** Change accelerometer measurement noise covariance.
   *
   * \param covariance New covariance.
   *
   */
  inline void accelNoiseCovariance(double covariance)
  {
    acceleroSensorCovariance_ = so::Matrix3::Identity() * covariance;
    updateNoiseCovariance();
  }

  /** Set debug flag.
   *
   * \param flag New debug flag.
   *
   */
  inline void debug(bool flag)
  {
    debug_ = flag;
  }

  /** Get force-sensor measurement noise covariance.
   *
   */
  inline double forceSensorNoiseCovariance() const
  {
    return contactSensorCovariance_(0, 0);
  }

  /** Change force-sensor measurement noise covariance.
   *
   * \param covariance New covariance.
   *
   */
  inline void forceSensorNoiseCovariance(double covariance)
  {
    contactSensorCovariance_.block<3, 3>(0, 0) = so::Matrix3::Identity() * covariance;
    updateNoiseCovariance();
  }

  /** Get gyrometer measurement noise covariance.
   *
   */
  inline double gyroNoiseCovariance() const
  {
    return gyroSensorCovariance_(0, 0);
  }

  /** Change gyrometer measurement noise covariance.
   *
   * \param covariance New covariance.
   *
   */
  inline void gyroNoiseCovariance(double covariance)
  {
    gyroSensorCovariance_ = so::Matrix3::Identity() * covariance;
    updateNoiseCovariance();
  }

  /*
  // Get last input vector sent to observer.
  inline const Eigen::VectorXd & inputs() const
  {
    return inputs_;
  }
  */

  /** Get last measurement vector sent to observer.
   *
   */
  inline const Eigen::VectorXd & measurements() const
  {
    return observer_.getEKF().getLastMeasurement();
  }

  /** Floating-base transform estimate.
   *
   */
  inline const sva::PTransformd & posW() const
  {
    return X_0_fb_;
  }

  /** Floating-base velocity estimate.
   *
   */
  inline const sva::MotionVecd & velW() const
  {
    return v_fb_0_;
  }

private:
  sva::PTransformd zeroPose_;
  sva::MotionVecd zeroMotion_;

  // so::kine::Kinematics worldFbKine_; // floating base in the user frame (world of the controller)
  // so::kine::Kinematics fbWorldKine_;
  so::kine::Kinematics worldCoMKine_;

  std::string category_ = "NaiveOdometry";
  /* custom list of robots to display */
  std::shared_ptr<mc_rbdyn::Robots> my_robots_;

  bool ekfIsSet_ = false;
  std::set<std::string> contactsFound_; // contacts found on each iteration

  Eigen::VectorXd res_;
  stateObservation::Vector6
      contactWrenchVector_; // vector shared by all the contacts that allows to build a (force+torque) wrench vector
                            // from the ForceSensor.wrench() function which returns a (torque+force) wrench vector

  unsigned noContact_ = 0;

  so::Vector correctedMeasurements_;
  so::kine::Kinematics globalCentroidKinematics_;
  Eigen::VectorXd predictedGlobalCentroidState_;
  std::vector<so::Vector> predictedAccelerometersGravityComponent_;
  std::vector<so::Vector> predictedWorldIMUsLinAcc_;
  std::vector<so::Vector> predictedAccelerometers_;

  double maxContactForceZ = 0; // allows to adapt the covariance on the contact based on how much set it is

  so::Vector innovation_;

  bool debug_ = false;
  bool verbose_ = true;

  double mass_ = 42; // [kg]
  stateObservation::KineticsObserver observer_;
  // std::set<std::string> contacts_; ///< Sorted list of contacts
  std::set<std::string> oldContacts_;
  MapContactsIMUs mapContacts_;
  MapContactsIMUs mapIMUs_;
  std::vector<sva::PTransformd>
      contactPositions_; ///< Position of the contact frames (force sensor frame when using force sensors)
  // sva::MotionVecd flexDamping_{{17, 17, 17}, {250, 250, 250}}; // HRP-4, {25.0, 200} for HRP-2

  // sva::MotionVecd flexStiffness_{{727, 727, 727}, {4e4, 4e4, 4e4}}; // HRP-4, {620, 3e5} for HRP-2
  sva::MotionVecd v_fb_0_ = sva::MotionVecd::Zero();
  sva::PTransformd X_0_fb_ = sva::PTransformd::Identity();
  sva::MotionVecd a_fb_0_ = sva::MotionVecd::Zero();

  sva::PTransformd accPos_; /**< currently hanled accelerometer pos in body */
  sva::PTransformd accContact_; /**< currently hanled contact pos in body */
  sva::RBInertiad inertiaWaist_; /**< grouped inertia */

  so::Vector3 additionalUserResultingForce_ = so::Vector3::Zero();
  so::Vector3 additionalUserResultingMoment_ = so::Vector3::Zero();

  std::unordered_map<std::string, ForceSignal> forceSignals;

  bool simStarted_ = false; // this variable is set to true when the robot touches the ground at the beginning of the
                            // simulation, allowing to start the estimaion at that time and not when the robot is still
                            // in the air, and therefore to avoid the big com's pose jump this transition involves

  /* Config variables */
  so::Matrix3 linStiffness_;
  so::Matrix3 linDamping_;

  so::Matrix3 angStiffness_;
  so::Matrix3 angDamping_;

  bool withOdometry_ = false;
  bool withFlatOdometry_ = false;

  bool withContactsDetection_ = true;
  bool withFilteredForcesContactDetection_ = false;
  bool withUnmodeledWrench_ = true;
  bool withGyroBias_ = true;

  int maxContacts_ = 4;
  int maxIMUs_ = 2;

  so::Quaternion robotImuOri_0;
  so::Quaternion realRobotImuOri_0;
  so::Quaternion robotImuOri_1;
  so::Quaternion realRobotImuOri_1;
  so::Quaternion robotFbOri_;
  so::Quaternion realRobotFbOri_;
  so::Vector3 robotPosImuInFB_0;
  so::Vector3 robotPosImuInFB_1;
  so::Vector3 realRobotPosImuInFB_0;
  so::Vector3 realRobotPosImuInFB_1;
  so::Vector3 robotTilt_0;
  so::Vector3 realRobotTilt_0;
  so::Vector3 MCKOrobotTilt_0;
  so::Vector3 robotTilt_1;
  so::Vector3 realRobotTilt_1;
  so::Vector3 MCKOrobotTilt_1;
  so::Quaternion realRobot_centroidImuOri_0;
  so::Quaternion realRobot_centroidImuOri_1;

  so::Matrix3 statePositionInitCovariance_;
  so::Matrix3 stateOriInitCovariance_;
  so::Matrix3 stateLinVelInitCovariance_;
  so::Matrix3 stateAngVelInitCovariance_;
  so::Matrix3 gyroBiasInitCovariance_;
  so::Matrix6 unmodeledWrenchInitCovariance_;
  so::Matrix12 contactInitCovarianceFirstContacts_;
  so::Matrix12 contactInitCovarianceNewContacts_;

  so::Matrix3 statePositionProcessCovariance_;
  so::Matrix3 stateOriProcessCovariance_;
  so::Matrix3 stateLinVelProcessCovariance_;
  so::Matrix3 stateAngVelProcessCovariance_;
  so::Matrix3 gyroBiasProcessCovariance_;
  so::Matrix6 unmodeledWrenchProcessCovariance_;
  so::Matrix12 contactProcessCovariance_;

  so::Matrix3 positionSensorCovariance_;
  so::Matrix3 orientationSensorCoVariance_;
  so::Matrix3 acceleroSensorCovariance_;
  so::Matrix3 gyroSensorCovariance_;
  so::Matrix6 contactSensorCovariance_;

  /* Config variables */
};

} // namespace mc_state_observation