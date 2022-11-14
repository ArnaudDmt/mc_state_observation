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


  class MapContactsIMU {
    /*
    Care with the use of getNameFromNum() : the mapping remains the same but the list of contacts returned buy the controller might differ over time 
    (contacts broken, etc) and the id of a contact in this list might not match with its rank in the controller's list. 
    Use this function preferably only to perform tasks performed on all the contacts and that require their name.
    */

  public:
    
    inline int getNumFromName(std::string name)
    {
      BOOST_ASSERT(mapNameNum.find(name) != mapNameNum.end() && "This id isn't attributed");
      return mapNameNum.find(name)->second;
    }

    inline void insertPair(std::string name) {
      if (mapNameNum.find(name) != mapNameNum.end()) return;
      
      mapNameNum.insert(std::pair<std::string,int>(name, num));
      num++;
    }
    
  private: 
    std::map<std::string,int> mapNameNum;
    int num = 0;

  };

  struct MCKineticsObserver : public mc_observers::Observer
  {

    MCKineticsObserver(const std::string & type, double dt);

    void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

    void reset(const mc_control::MCController & ctl) override;

    bool run(const mc_control::MCController & ctl) override;

    void update(mc_control::MCController & ctl) override;

protected:
  void update(mc_rbdyn::Robot & robot);

  void initObserverStateVector(const mc_rbdyn::Robot & robot);

  void updateIMUs(const mc_rbdyn::Robot & robot);

  /*! \brief Add observer from logger
   *
   * @param category Category in which to log this observer
   */
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

  void updateContacts(const mc_rbdyn::Robot& robot, std::set<std::string> contacts);

protected:
  std::string robot_ = "";
  //std::string imuSensor_ = "";
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
      return acceleroSensorCovariance_(0,0);
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
      return contactSensorCovariance_(0,0);
    }

    /** Change force-sensor measurement noise covariance.
     *
     * \param covariance New covariance.
     *
     */
    inline void forceSensorNoiseCovariance(double covariance)
    {
      contactSensorCovariance_.block<3, 3>(0, 0) =  so::Matrix3::Identity() * covariance;
      updateNoiseCovariance();
    }

    /** Get gyrometer measurement noise covariance.
     *
     */
    inline double gyroNoiseCovariance() const
    {
      return gyroSensorCovariance_(0,0);
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
    std::shared_ptr<mc_rbdyn::Robots> my_robots_;

    bool ekfIsSet_ = false;

    Eigen::VectorXd res_;
    stateObservation::Vector6 contactWrenchVector_; // vector shared by all the contacts that allows to build a (force+torque) wrench vector 
                                                  // from the ForceSensor.wrench() function which returns a (torque+force) wrench vector


    so::Vector correctedMeasurements_;
    so::kine::Kinematics globalCentroidKinematics_;
    so::kine::Kinematics predictedGlobalCentroidKinematics_;
    std::vector<so::Vector> predictedAccelerometersGravityComponent_;
    std::vector<so::Vector> predictedWorldIMUsLinAcc_;

    so::Vector innovation_;

    bool debug_ = false;
    bool verbose_ = true;
    
    double mass_ = 42; // [kg]
    stateObservation::KineticsObserver observer_;
    std::set<std::string> contacts_; ///< Sorted list of contacts
    MapContactsIMU mapContacts_;
    MapContactsIMU mapIMUs_;
    std::vector<sva::PTransformd> contactPositions_; ///< Position of the contact frames (force sensor frame when using force sensors)
    //sva::MotionVecd flexDamping_{{17, 17, 17}, {250, 250, 250}}; // HRP-4, {25.0, 200} for HRP-2
    
    // sva::MotionVecd flexStiffness_{{727, 727, 727}, {4e4, 4e4, 4e4}}; // HRP-4, {620, 3e5} for HRP-2
    sva::MotionVecd v_fb_0_ = sva::MotionVecd::Zero();
    sva::PTransformd X_0_fb_ = sva::PTransformd::Identity();
    sva::PTransformd accPos_; /**< currently hanled accelerometer pos in body */
    sva::PTransformd accContact_; /**< currently hanled contact pos in body */
    sva::RBInertiad inertiaWaist_; /**< grouped inertia */

    bool simStarted_ = false; // this variable is set to true when the robot touches the ground at the beginning of the simulation, 
                              // allowing to start the estimaion at that time and not when the robot is still in the air, 
                              // and therefore to avoid the big com's pose jump this transition involves


    /* Config variables */
    sva::MotionVecd flexStiffness_;
    sva::MotionVecd flexDamping_;

    int maxContacts_ = 2;
    int maxIMUs_ = 2;

    so::Matrix3 statePositionInitCovariance_;
    so::Matrix3 stateOriInitCovariance_;
    so::Matrix3 stateLinVelInitCovariance_;
    so::Matrix3 stateAngVelInitCovariance_;
    so::Matrix3 gyroBiasInitCovariance_;
    so::Matrix3 unmodeledWrenchInitCovariance_;
    so::Matrix12 contactInitCovariance_;

    so::Matrix3 statePositionProcessCovariance_;
    so::Matrix3 stateOriProcessCovariance_;
    so::Matrix3 stateLinVelProcessCovariance_;
    so::Matrix3 stateAngVelProcessCovariance_;
    so::Matrix3 gyroBiasProcessCovariance_;
    so::Matrix3 unmodeledWrenchProcessCovariance_;
    so::Matrix12 contactProcessCovariance_;

    so::Matrix3 positionSensorCovariance_;
    so::Matrix3 orientationSensorCoVariance_;
    so::Matrix3 acceleroSensorCovariance_;
    so::Matrix3 gyroSensorCovariance_;
    so::Matrix6 contactSensorCovariance_;

    /* Config variables */

  };

} // mc_state_observation

