/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#pragma once

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Robot.h>

#include <state-observation/dynamics-estimators/kinetics-observer.hpp>

#include <mc_observers/Observer.h>

namespace mc_state_observation
{
  /** Flexibility observer from:
   *
   *    "Tilt estimator for 3D non-rigid pendulum based on a tri-axial
   *    accelerometer and gyrometer". Mehdi Benallegue, Abdelaziz Benallegue,
   *    Yacine Chitour. IEEE-RAS Humanoids 2017. <hal-01499167>
   *
   */


  class MapContactsIMU{

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
      return accelNoiseCovariance_;
    }

    /** Change accelerometer measurement noise covariance.
     *
     * \param covariance New covariance.
     *
     */
    inline void accelNoiseCovariance(double covariance)
    {
      accelNoiseCovariance_ = covariance;
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
      return forceSensorNoiseCovariance_;
    }

    /** Change force-sensor measurement noise covariance.
     *
     * \param covariance New covariance.
     *
     */
    inline void forceSensorNoiseCovariance(double covariance)
    {
      forceSensorNoiseCovariance_ = covariance;
      updateNoiseCovariance();
    }

    /** Get gyrometer measurement noise covariance.
     *
     */
    inline double gyroNoiseCovariance() const
    {
      return gyroNoiseCovariance_;
    }

    /** Change gyrometer measurement noise covariance.
     *
     * \param covariance New covariance.
     *
     */
    inline void gyroNoiseCovariance(double covariance)
    {
      gyroNoiseCovariance_ = covariance;
      updateNoiseCovariance();
    }

    /** Get last input vector sent to observer.
     *
     */
    inline const Eigen::VectorXd & inputs() const
    {
      return inputs_;
    }

    /** Get last measurement vector sent to observer.
     *
     */
    inline const Eigen::VectorXd & measurements() const
    {
      return measurements_;
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
    Eigen::VectorXd inputs_;
    Eigen::VectorXd measurements_;
    Eigen::VectorXd res_;
    bool debug_ = false;
    bool verbose_ = true;
    double accelNoiseCovariance_ = 1e-4;
    double forceSensorNoiseCovariance_ = 5.; // from https://gite.lirmm.fr/caron/mc_observers/issues/1#note_10040
    double gyroNoiseCovariance_ = 1e-9;
    double mass_ = 42; // [kg]
    stateObservation::KineticsObserver observer_;
    std::set<std::string> contacts_; ///< Sorted list of contacts
    MapContactsIMU mapContacts_;
    MapContactsIMU mapIMUs_;
    std::vector<sva::PTransformd> contactPositions_; ///< Position of the contact frames (force sensor frame when using force sensors)
    sva::MotionVecd flexDamping_{{17, 17, 17}, {250, 250, 250}}; // HRP-4, {25.0, 200} for HRP-2
    sva::MotionVecd flexStiffness_{{727, 727, 727}, {4e4, 4e4, 4e4}}; // HRP-4, {620, 3e5} for HRP-2
    sva::MotionVecd contactDamping_{{0.17, 0.17, 0.17}, {2.50, 2.50, 2.50}}; // HRP-4, {25.0, 200} for HRP-2
    sva::MotionVecd contactStiffness_{{7.27, 7.27, 7.27}, {4e1, 4e1, 4e1}}; // HRP-4, {620, 3e5} for HRP-2
    sva::MotionVecd v_fb_0_ = sva::MotionVecd::Zero();
    sva::PTransformd X_0_fb_ = sva::PTransformd::Identity();
    sva::PTransformd accPos_; /**< currently hanled accelerometer pos in body */
    sva::PTransformd accContact_; /**< currently hanled contact pos in body */
    sva::RBInertiad inertiaWaist_; /**< grouped inertia */

    bool simStarted_ = false;
  };

} // mc_state_observation

