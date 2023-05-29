/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#pragma once

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Robot.h>

#include <state-observation/dynamics-estimators/kinetics-observer.hpp>

#include <mc_observers/Observer.h>
#include <mc_state_observation/svaKinematicsConversion.h>


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

  struct Sensor
  {

  public:
    inline const int & getID()
    {
      return id_;
    }
    inline const std::string & getName()
    {
      return name_;
    }

  protected:
    Sensor() {}
    ~Sensor() {}
    Sensor(int id, std::string name) : id_(id), name_(name) {}

  protected:
    int id_;
    std::string name_;
  };

  struct IMU : virtual public Sensor
  {
  public:
    IMU(int id, std::string name)
    {
      id_ = id;
      name_ = name;
      gyroBias << 0.0, 0.0, 0.0;
    }
    ~IMU() {}

  public:
    so::Vector3 gyroBias;
  };

  struct Contact : virtual public Sensor
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  protected:
    Contact() {}
    ~Contact() {}
    Contact(int id, std::string name)
    {
      id_ = id;
      name_ = name;
      resetContact();
    }

  public:
    inline void resetContact()
    {
      wasAlreadySet = false;
      isSet = false;
      zmp.set(false);
    }
    inline const so::Vector3 & getZMP()
    {
      return zmp;
    }

  public:
    bool isSet = false;
    bool wasAlreadySet = false;
    so::CheckedVector3 zmp;
  };

  struct ContactWithSensor : virtual public Contact
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
    ContactWithSensor(int id, std::string name)
    {
      id_ = id;
      name_ = name;
      resetContact();
    }
    ~ContactWithSensor() {}
    inline void resetContact()
    {
      wasAlreadySet = false;
      isSet = false;
      sensorWasEnabled = false;

      // also filtered force? see when this feature will be corrected
    }

  public:
    so::Vector6 wrenchInCentroid = so::Vector6::Zero(); // for debug only
    double normForce = 0.0; // for debug only
    bool isExternalWrench = true;
    bool sensorEnabled = true;
    bool sensorWasEnabled = false; // allows to know if the contact's measurements have to be added during the update

    bool isAttachedToSurface = true;
    std::string surface;

    /* Force filtering for the contact detection */
    so::Vector3 filteredForce = so::Vector3::Zero();
    double lambda = 0.0;
  };

  struct ContactWithoutSensor : virtual public Contact
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
    ContactWithoutSensor(int id, std::string name)
    {
      id_ = id;
      name_ = name;
    }
    ~ContactWithoutSensor() {}
  };

  struct MapContacts
  {
  public:
    inline ContactWithSensor & contactWithSensor(const std::string & name)
    {
      return mapContactsWithSensors_.at(name);
    }

    inline ContactWithSensor & contactWithSensor(const int & num)
    {
      return mapContactsWithSensors_.at(getNameFromNum(num));
    }

    inline ContactWithoutSensor & contactWithoutSensor(const std::string & name)
    {
      return mapContactsWithoutSensors_.at(name);
    }

    inline ContactWithoutSensor & contactWithoutSensor(const int & num)
    {
      return mapContactsWithoutSensors_.at(getNameFromNum(num));
    }

    inline std::map<std::string, ContactWithSensor> & contactsWithSensors()
    {
      return mapContactsWithSensors_;
    }

    inline std::map<std::string, ContactWithoutSensor> & contactsWithoutSensors()
    {
      return mapContactsWithoutSensors_;
    }

    inline const std::string & getNameFromNum(const int & num)
    {
      return insertOrder.at(num);
    }

    inline const std::vector<std::string> & getList()
    {
      return insertOrder;
    }

    inline bool hasSensor(const std::string & element)
    {
      BOOST_ASSERT(hasElement(element) && "This contact does not belong to the list.");
      return hasSensor_.at(element);
    }

    inline const int & getNumFromName(const std::string & name)
    {
      if(hasSensor_.at(name))
      {
        return mapContactsWithSensors_.at(name).getID();
      }
      else
      {
        return mapContactsWithoutSensors_.at(name).getID();
      }
    }

    inline const so::Vector3 & getZMPFromName(const std::string & name)
    {
      if(hasSensor_.at(name))
      {
        return mapContactsWithSensors_.at(name).getZMP();
      }
      else
      {
        return mapContactsWithoutSensors_.at(name).getZMP();
      }
    }

    inline bool hasElement(const std::string & element)
    {
      return hasSensor_.find(element) != hasSensor_.end();
    }

    inline void insertContact(const std::string & name, const bool & hasSensor)
    {
      if(checkAlreadyExists(name, hasSensor)) return;
      insertOrder.push_back(name);
      insertElement(name, hasSensor);

      num++;
    }

  private:
    inline void insertElement(const std::string & name, const bool & hasSensor)
    {
      if(hasSensor)
      {
        mapContactsWithSensors_.insert(std::make_pair(name, ContactWithSensor(num, name)));
        hasSensor_.insert(std::make_pair(name, true));
      }
      else
      {
        mapContactsWithoutSensors_.insert(std::make_pair(name, ContactWithoutSensor(num, name)));
        hasSensor_.insert(std::make_pair(name, true));
      }
    }
    inline virtual bool checkAlreadyExists(const std::string & name, const bool & hasContact)
    {
      if(hasSensor_.find(name) != hasSensor_.end())
      {
        if(hasContact)
        {
          BOOST_ASSERT((mapContactsWithoutSensors_.find(name) == mapContactsWithoutSensors_.end())
                       && "The contact already exists but was associated to no sensor");
          return true;
        }
        else
        {
          BOOST_ASSERT((mapContactsWithSensors_.find(name) == mapContactsWithSensors_.end())
                       && "The contact already exists but was associated to a sensor");
          return true;
        }
      }
      else
      {
        return false;
      }
    }

  private:
    std::map<std::string, bool>
        hasSensor_; // map containing all the contacts and indicating if each sensor has a contact or not
    std::map<std::string, ContactWithSensor> mapContactsWithSensors_;
    std::map<std::string, ContactWithoutSensor> mapContactsWithoutSensors_;
    std::vector<std::string> insertOrder;
    int num = 0;
  };

  struct MapIMUs
  {
  public:
    inline const int & getNumFromName(const std::string & name)
    {
      return mapIMUs_.find(name)->second.getID();
    }
    inline bool hasIMU(const std::string & element)
    {
      return mapIMUs_.find(element) != mapIMUs_.end();
    }

    inline const std::string & getNameFromNum(const int & num)
    {
      return insertOrder.at(num);
    }

    inline const std::vector<std::string> & getList()
    {
      return insertOrder;
    }
    inline virtual bool hasElement(const std::string & name)
    {
      checkAlreadyExists(name);
    }

    inline void insertIMU(std::string name)
    {
      if(checkAlreadyExists(name)) return;
      insertOrder.push_back(name);
      mapIMUs_.insert(std::make_pair(name, IMU(num, name)));
      num++;
    }

    inline IMU & operator()(std::string name)
    {
      BOOST_ASSERT(checkAlreadyExists(name) && "The requested sensor doesn't exist");
      return mapIMUs_.at(name);
    }

  private:
    inline virtual bool checkAlreadyExists(const std::string & name)
    {
      if(mapIMUs_.find(name) != mapIMUs_.end())
        return true;
      else
        return false;
    }

  private:
    std::vector<std::string> insertOrder;
    std::map<std::string, IMU> mapIMUs_;
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

    void addContactLogEntries(mc_rtc::Logger & logger, const std::string & contactName);

    void removeContactLogEntries(mc_rtc::Logger & logger, const std::string & contactName);

    void addContactMeasurementsLogEntries(mc_rtc::Logger & logger, const std::string & contactName);

    void removeContactMeasurementsLogEntries(mc_rtc::Logger & logger, const std::string & contactName);

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
    const std::set<std::string> & findContacts(const mc_control::MCController & solver);
    const std::set<std::string> & findContactsFromSolver(const mc_control::MCController & solver);
    const std::set<std::string> & findContactsFromSurfaces(const mc_control::MCController & solver);
    const std::set<std::string> & findContactsFromThreshold(const mc_control::MCController & solver);

    void updateContacts(const mc_control::MCController & ctl,
                        std::set<std::string> contacts,
                        mc_rtc::Logger & logger);
    void updateContact(const mc_control::MCController & ctl,
                       const std::string & name,
                       mc_rtc::Logger & logger);

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
    std::vector<std::string> surfacesForContactDetection_;

    double gyroBiasStandardDeviation_ = 0.0;
    std::vector<std::string> contactsSensorDisabledInit_;

    so::Vector3 totalForceCentroid_ = so::Vector3::Zero();
    so::Vector3 totalTorqueCentroid_ = so::Vector3::Zero();

    sva::PTransformd zeroPose_;
    sva::MotionVecd zeroMotion_;

    // so::kine::Kinematics worldFbKine_; // floating base in the user frame (world of the controller)
    // so::kine::Kinematics fbWorldKine_;
    so::kine::Kinematics worldCoMKine_;

    std::string category_ = "MCKineticsObserver";
    /* custom list of robots to display */
    std::shared_ptr<mc_rbdyn::Robots> my_robots_;

    bool ekfIsSet_ = false;
    std::set<std::string> contactsFound_; //contacts found on each iteration

    Eigen::VectorXd res_;
    stateObservation::Vector6 contactWrenchVector_; // vector shared by all the contacts that allows to build a (force+torque) wrench vector 
                                                  // from the ForceSensor.wrench() function which returns a (torque+force) wrench vector

    so::Vector correctedMeasurements_;
    so::kine::Kinematics globalCentroidKinematics_;
    Eigen::VectorXd predictedGlobalCentroidState_;
    std::vector<so::Vector> predictedAccelerometersGravityComponent_;
    std::vector<so::Vector> predictedWorldIMUsLinAcc_;
    std::vector<so::Vector> predictedAccelerometers_;

    double contactDetectionPropThreshold_ = 0.0;
    double contactDetectionThreshold_ = 0.0;

    so::Vector innovation_;

    bool debug_ = false;
    bool verbose_ = true;
    
    double mass_ = 42; // [kg]
    stateObservation::KineticsObserver observer_;
    //std::set<std::string> contacts_; ///< Sorted list of contacts
    std::set<std::string> oldContacts_;
    MapContacts mapContacts_; // contacts are detected in findContacts() function. They are detected from force
                              // sensors so their names corresponds to the name of the associated force sensors. Has
                              // to be changed if one wants to works with contacts without force sensors.
    MapIMUs mapIMUs_;
    std::vector<sva::PTransformd> contactPositions_; ///< Position of the contact frames (force sensor frame when using force sensors)
    //sva::MotionVecd flexDamping_{{17, 17, 17}, {250, 250, 250}}; // HRP-4, {25.0, 200} for HRP-2
    
    // sva::MotionVecd flexStiffness_{{727, 727, 727}, {4e4, 4e4, 4e4}}; // HRP-4, {620, 3e5} for HRP-2
    sva::MotionVecd v_fb_0_ = sva::MotionVecd::Zero();
    sva::PTransformd X_0_fb_ = sva::PTransformd::Identity();
    sva::MotionVecd a_fb_0_ = sva::MotionVecd::Zero();

    sva::PTransformd accPos_; /**< currently hanled accelerometer pos in body */
    sva::PTransformd accContact_; /**< currently hanled contact pos in body */
    sva::RBInertiad inertiaWaist_; /**< grouped inertia */

    so::Vector3 additionalUserResultingForce_ = so::Vector3::Zero();
    so::Vector3 additionalUserResultingMoment_ = so::Vector3::Zero();

    bool simStarted_ = false; // this variable is set to true when the robot touches the ground at the beginning of the simulation, 
                              // allowing to start the estimaion at that time and not when the robot is still in the air, 
                              // and therefore to avoid the big com's pose jump this transition involves


    /* Config variables */
    so::Matrix3 linStiffness_;
    so::Matrix3 linDamping_;

    so::Matrix3 angStiffness_;
    so::Matrix3 angDamping_;

    bool withDebugLogs_ = false;
    std::string leftHandDetection_ = "asContact";
    bool withOdometry_ = false;
    bool withFlatOdometry_ = false;

    std::string contactsDetection_;
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

} // mc_state_observation
