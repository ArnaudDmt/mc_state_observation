/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#pragma once

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Robot.h>

#include <state-observation/dynamics-estimators/kinetics-observer.hpp>

#include <mc_observers/Observer.h>
#include <mc_state_observation/svaKinematicsConversion.h>


namespace mc_state_observation
{
  /** Interface for the use of the Kinetics Observer within mc_rtc: \n
   * The Kinetics Observer requires inputs expressed in the frame of the floating base. It then performs a conversion to
   *the centroid frame, a frame located at the center of mass of the robot and with the orientation of the floating
   *base of the real robot.
   *The inputs are obtained from a robot called the inputRobot. Its configuration is the one of real robot, but
   *its floating base's frame is superimposed with the world frame. This allows to ease computations performed in the
   *local frame of the robot.
   **/

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

  /* Contains the important variables associated to the IMU */
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
    stateObservation::Vector3 gyroBias;
  };

  /* Contains the important variables associated to the contact */
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
    inline const stateObservation::Vector3 & getZMP()
    {
      return zmp;
    }

  public:
    bool isSet = false;
    bool wasAlreadySet = false;
    stateObservation::CheckedVector3 zmp;
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
    stateObservation::Vector6 wrenchInCentroid = stateObservation::Vector6::Zero(); // for debug only
    double normForce = 0.0; // for debug only
    // the measurement of the sensor has to be considered as an input for the Kinetics Observer
    bool isExternalWrench = true;
    // the sensor measurement have to be used in the correction by the Kinetics Observer
    bool sensorEnabled = true;
    // allows to know if the contact's measurements have to be added during the update
    bool sensorWasEnabled = false;

    // indicates if the sensor is directly attached to a surface (true) or not (false). Default is true because in the
    // case of detection of contacts by thresholding the measured force (@contactsDetection_ = fromThreshold), we cannot
    // know precisely the surface of contact, so we will consider that the kinematics of the contact surface are the
    // ones of the sensor
    bool sensorAttachedToSurface = true;
    // surface of contact
    std::string surface;

    /* Force filtering for the contact detection */
    stateObservation::Vector3 filteredForce = stateObservation::Vector3::Zero();
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
    /// @brief Accessor for the a contact associated to a sensor contained in the map
    ///
    /// @param name The name of the contact to access
    /// @return ContactWithSensor&
    inline ContactWithSensor & contactWithSensor(const std::string & name)
    {
      return mapContactsWithSensors_.at(name);
    }
    /// @brief Accessor for the a contact associated to a sensor contained in the map
    ///
    /// @param num The index of the contact to access
    /// @return ContactWithSensor&
    inline ContactWithSensor & contactWithSensor(const int & num)
    {
      return mapContactsWithSensors_.at(getNameFromNum(num));
    }

    /// @brief Accessor for the a contact that is not associated to a sensor contained in the map
    /// @param name The name of the contact to access
    /// @return ContactWithoutSensor&
    inline ContactWithoutSensor & contactWithoutSensor(const std::string & name)
    {
      return mapContactsWithoutSensors_.at(name);
    }

    /// @brief Accessor for the a contact that is not associated to a sensor contained in the map
    /// @param num The index of the contact to access
    /// @return ContactWithoutSensor&
    inline ContactWithoutSensor & contactWithoutSensor(const int & num)
    {
      return mapContactsWithoutSensors_.at(getNameFromNum(num));
    }

    /// @brief Get the map of all the contacts associated to a sensor
    ///
    /// @return std::map<std::string, ContactWithSensor>&
    inline std::map<std::string, ContactWithSensor> & contactsWithSensors()
    {
      return mapContactsWithSensors_;
    }
    /// @brief Get the map of all the contacts that are not associated to a sensor
    ///
    /// @return std::map<std::string, ContactWithSensor>&
    inline std::map<std::string, ContactWithoutSensor> & contactsWithoutSensors()
    {
      return mapContactsWithoutSensors_;
    }

    /// @brief Get the list of all the contacts (with and without sensors)
    ///
    /// @return const std::vector<std::string> &
    inline const std::vector<std::string> & getList()
    {
      return insertOrder;
    }

    /// @brief Returns true if the contact is associated to a sensor
    ///
    /// @param name The index of the contact to access
    /// @return bool
    inline bool hasSensor(const std::string & element)
    {
      BOOST_ASSERT(hasElement(element) && "This contact does not belong to the list.");
      return hasSensor_.at(element);
    }

    /// @brief Get the name of a contact given its index
    ///
    /// @param num The index of the contact
    /// @return const std::string &
    inline const std::string & getNameFromNum(const int & num)
    {
      return insertOrder.at(num);
    }

    /// @brief Get the index of a contact given its name
    ///
    /// @param name The name of the contact
    /// @return const int &
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

    /// @brief Get the measured zmp of a contact given its name
    ///
    /// @param name The name of the contact
    /// @return const stateObservation::Vector3 &
    inline const stateObservation::Vector3 & getZMPFromName(const std::string & name)
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

    /// @brief Checks if the given contact exists
    ///
    /// @param element The name of the contact
    /// @return bool
    inline bool hasElement(const std::string & element)
    {
      return hasSensor_.find(element) != hasSensor_.end();
    }

    /// @brief Check that a contact still does not exist, if so, insert a contact to the map of contacts. The contact
    /// can either be associated to a sensor or not.
    ///
    /// @param element The name of the contact
    /// @param hasSensor True if the contact is attached to a sensor.
    inline void insertContact(const std::string & name, const bool & hasSensor)
    {
      if(checkAlreadyExists(name, hasSensor)) return;
      insertOrder.push_back(name);
      insertElement(name, hasSensor);

      num++;
    }

  private:
    /// @brief Insert a contact to the map of contacts. The contact can either be associated to a sensor or not.
    ///
    /// @param element The name of the contact
    /// @param hasSensor True if the contact is attached to a sensor.
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

    /// @brief Check if a contact already exists in the list. If it already exists, checks that the association to a
    /// sensor remained unchanged.
    ///
    /// @param element The name of the contact
    /// @param hasSensor True if the contact is attached to a sensor.
    /// @return bool
    inline bool checkAlreadyExists(const std::string & name, const bool & hasContact)
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
    // map containing all the contacts and indicating if each sensor has a contact or not
    std::map<std::string, bool> hasSensor_;
    // map containing all the contacts associated to a sensor
    std::map<std::string, ContactWithSensor> mapContactsWithSensors_;
    // map containing all the contacts that are not associated to a sensor
    std::map<std::string, ContactWithoutSensor> mapContactsWithoutSensors_;
    // List of all the contacts
    std::vector<std::string> insertOrder;
    // Index generator, incremented everytime a new contact is created
    int num = 0;
  };

  struct MapIMUs
  {
  public:
    /// @brief Get the index of the IMU given its name.
    /// @param name The name of the IMU
    /// @return const int &
    inline const int & getNumFromName(const std::string & name)
    {
      return mapIMUs_.find(name)->second.getID();
    }
    /// @brief Get the name of the IMU given its index.
    /// @param num The index of the IMU
    /// @return const std::string &
    inline const std::string & getNameFromNum(const int & num)
    {
      return insertOrder.at(num);
    }

    /// @brief Get the list of all the IMUs.
    /// @return const std::vector<std::string> &
    inline const std::vector<std::string> & getList()
    {
      return insertOrder;
    }

    /// @brief Checks if the required IMU exists.
    /// @param name The name of the IMU
    /// @return bool
    inline bool hasElement(const std::string & name)
    {
      return checkAlreadyExists(name);
    }

    /// @brief Inserts an IMU to the map.
    /// @param name The name of the IMU
    inline void insertIMU(std::string name)
    {
      if(checkAlreadyExists(name)) return;
      insertOrder.push_back(name);
      mapIMUs_.insert(std::make_pair(name, IMU(num, name)));
      num++;
    }

    /// @brief Accessor for an IMU in the list.
    /// @param name The name of the IMU
    /// @return bool
    inline IMU & operator()(std::string name)
    {
      BOOST_ASSERT(checkAlreadyExists(name) && "The requested sensor doesn't exist");
      return mapIMUs_.at(name);
    }

  private:
    /// @brief Checks if the required IMU exists.
    /// @param name The name of the IMU
    /// @return bool
    inline bool checkAlreadyExists(const std::string & name)
    {
      return mapIMUs_.find(name) != mapIMUs_.end();
    }

  private:
    // list of all the IMUs.
    std::vector<std::string> insertOrder;
    // map associating all the IMUs to their names.
    std::map<std::string, IMU> mapIMUs_;
    // Index generator, incremented everytime a new IMU is added
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
    /// @brief Update the pose and velocities of the robot in the world frame. Used only to update the ones of the robot
    /// used for the visualization of the estimation made by the Kinetics Observer.
    /// @param robot The robot to update.
    void update(mc_rbdyn::Robot & robot);

    /// @brief Initializer for the Kinetics Observer's state vector
    /// @param robot The control robot
    void initObserverStateVector(const mc_rbdyn::Robot & robot);

    /// @brief Sums up the wrenches measured by the unused force sensors expressed in the centroid frame to give them as
    /// an input to the Kinetics Observer
    /// @param inputRobot A robot whose configuration is the one of real robot, but whose pose, velocities and
    /// accelerations are set to zero in the control frame. Allows to ease computations performed in the local frame of
    /// the robot.
    /// @param measRobot The control robot
    void inputAdditionalWrench(const mc_rbdyn::Robot & inputRobot, const mc_rbdyn::Robot & measRobot);

    /// @brief Update the IMUs, including the measurements, measurement covariances and kinematics in the floating
    /// base's frame (user frame)
    /// @param measRobot The control robot
    /// @param inputRobot A robot whose configuration is the one of real robot, but whose pose, velocities and
    /// accelerations are set to zero in the control frame. Allows to ease computations performed in the local frame of
    /// the robot.
    void updateIMUs(const mc_rbdyn::Robot & measRobot, const mc_rbdyn::Robot & inputRobot);

    /*! \brief Add observer from logger
     *
     * @param category Category in which to log this observer
     */

    void plotVariablesBeforeUpdate(const mc_control::MCController & ctl, mc_rtc::Logger & logger);

    void plotVariablesAfterUpdate(mc_rtc::Logger & logger);

    /// @brief Add the logs of the desired contact.
    /// @param contactName The name of the contact.
    /// @param logger
    void addContactLogEntries(mc_rtc::Logger & logger, const std::string & contactName);
    /// @brief Remove the logs of the desired contact.
    /// @param contactName The name of the contact.
    /// @param logger
    void removeContactLogEntries(mc_rtc::Logger & logger, const std::string & contactName);

    /// @brief Add the measurements logs of the desired contact.
    /// @param contactName The name of the contact.
    /// @param logger
    void addContactMeasurementsLogEntries(mc_rtc::Logger & logger, const std::string & contactName);
    /// @brief Remove the measurements logs of the desired contact.
    /// @param contactName The name of the contact.
    /// @param logger
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
    /// @brief Updates the list of currently set contacts.
    /// @details The contact detection is defined by the variable
    /// @contactsDetection_. The possible values of @contactsDetection_ are: \n fromSolver: The contacts are given by
    /// the controller directly (then thresholded based on the measured force). \n fromSurfaces: The contacts are given
    /// by the controller directly from a surface (then thresholded based on the measured force). \n fromThreshold: The
    /// contacts are not required to be given by the controller (the detection is based on a thresholding of the
    /// measured force)
    /// @return std::set<std::string> &
    const std::set<std::string> & findContacts(const mc_control::MCController & ctl);
    /// @brief Updates the list @contactsFound_ of currently set contacts directly from the controller.
    /// @details Called by \ref findContacts(const mc_control::MCController & ctl) if @contactsDetection_ is equal to
    /// "fromSolver". The contacts are given by the controller directly (then thresholded based on the measured force).
    /// @return std::set<std::string> &
    void findContactsFromSolver(const mc_control::MCController & ctl);
    /// @brief Updates the list @contactsFound_ of currently set contacts from the surfaces given by the controller.
    /// @details Called by \ref findContacts(const mc_control::MCController & ctl) if @contactsDetection_ is equal to
    /// "fromSurfaces". The contacts are given by the controller directly from a surface (then thresholded based on the
    /// measured force).
    /// @return std::set<std::string> &
    void findContactsFromSurfaces(const mc_control::MCController & ctl);
    /// @brief Updates the list @contactsFound_ of currently set contacts from a thresholding of the measured forces.
    /// @details Called by \ref findContacts(const mc_control::MCController & ctl) if @contactsDetection_ is equal to
    /// "fromThreshold". The contacts are not required to be given by the controller (the detection is based on a
    /// thresholding of the measured force).
    /// @return std::set<std::string> &
    void findContactsFromThreshold(const mc_control::MCController & ctl);

    /// @brief Update the currently set contacts.
    /// @details The list of contacts is returned by \ref findContacts(const mc_control::MCController & ctl). Calls \ref
    /// updateContact(const mc_control::MCController & ctl, const std::string & name, mc_rtc::Logger & logger).
    /// @param contacts The list of contacts returned by \ref findContacts(const mc_control::MCController & ctl).
    void updateContacts(const mc_control::MCController & ctl,
                        std::set<std::string> contacts,
                        mc_rtc::Logger & logger);
    /// @brief Update the contact or create it if it still does not exist.
    /// @details Called by \ref updateContacts(const mc_control::MCController & ctl, std::set<std::string> contacts,
    /// mc_rtc::Logger & logger).
    /// @param name The name of the contact to update.
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
      acceleroSensorCovariance_ = stateObservation::Matrix3::Identity() * covariance;
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
      contactSensorCovariance_.block<3, 3>(0, 0) =  stateObservation::Matrix3::Identity() * covariance;
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
      gyroSensorCovariance_ = stateObservation::Matrix3::Identity() * covariance;
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
    inline const Eigen::VectorXd measurements() const
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
    Eigen::VectorXd predictedGlobalCentroidState_;
    std::vector<stateObservation::Vector> predictedAccelerometersGravityComponent_;
    std::vector<stateObservation::Vector> predictedWorldIMUsLinAcc_;
    std::vector<stateObservation::Vector> predictedAccelerometers_;

    // list of surfaces used for contacts detection if @contactsDetection_ is set to "fromSurfaces"
    std::vector<std::string> surfacesForContactDetection_;
    // list of sensors that must not be used from the start of the observer

    double gyroBiasStandardDeviation_ = 0.0;
    std::vector<std::string> contactsSensorDisabledInit_;
    // zero frame transformation

    stateObservation::Vector3 totalForceCentroid_ = stateObservation::Vector3::Zero();
    stateObservation::Vector3 totalTorqueCentroid_ = stateObservation::Vector3::Zero();

    sva::PTransformd zeroPose_;
    // zero velocity or acceleration
    sva::MotionVecd zeroMotion_;

    // kinematics of the CoM within the world frame of the control robot
    stateObservation::kine::Kinematics worldCoMKine_;

    std::string category_ = "MCKineticsObserver";
    /* custom list of robots to display */
    std::shared_ptr<mc_rbdyn::Robots> my_robots_;

    // the Kinetics Observer completed the loop at least once
    bool ekfIsSet_ = false;

    // contacts found on each iteration
    std::set<std::string> contactsFound_;

    // state vector resulting from the Kinetics Observer esimation
    Eigen::VectorXd res_;

    // vector shared by all the contacts that allows to build a (force+torque) wrench vector from the
    // ForceSensor.wrench() function which returns a (torque+force) wrench vector
    stateObservation::Vector6 contactWrenchVector_;

    // rate to apply on the measured force to obtain the threshold for contact detection.
    double contactDetectionPropThreshold_ = 0.0;
    // threshold on the measured force for contact detection.
    double contactDetectionThreshold_ = 0.0;

    // For logs only. Prediction of the measurements from the newly corrected state
    stateObservation::Vector correctedMeasurements_;
    // For logs only. Kinematics of the centroid frame within the world frame
    stateObservation::kine::Kinematics globalCentroidKinematics_;

    bool debug_ = false;
    bool verbose_ = true;
    
    double mass_ = 42; // [kg]

    // instance of the Kinetics Observer
    stateObservation::KineticsObserver observer_;

    // List of previously set contacts
    std::set<std::string> oldContacts_;

    MapContacts mapContacts_;
    MapIMUs mapIMUs_;

    // velocity of the floating base within the world frame (real one, not the one of the control robot)
    sva::MotionVecd v_fb_0_ = sva::MotionVecd::Zero();
    // pose of the floating base within the world frame (real one, not the one of the control robot)
    sva::PTransformd X_0_fb_ = sva::PTransformd::Identity();
    // acceleration of the floating base within the world frame (real one, not the one of the control robot)
    sva::MotionVecd a_fb_0_ = sva::MotionVecd::Zero();

    sva::PTransformd accPos_; /**< currently hanled accelerometer pos in body */
    sva::PTransformd accContact_; /**< currently hanled contact pos in body */
    sva::RBInertiad inertiaWaist_; /**< grouped inertia */

    // total force measured by the sensors that are not associated to a currently set contact and expressed in the
    // floating base's frame. Used as an input for the Kinetics Observer.
    stateObservation::Vector3 additionalUserResultingForce_ = stateObservation::Vector3::Zero();
    // total torque measured by the sensors that are not associated to a currently set contact and expressed in the
    // floating base's frame. Used as an input for the Kinetics Observer.
    stateObservation::Vector3 additionalUserResultingMoment_ = stateObservation::Vector3::Zero();

    // this variable is set to true when the robot touches the ground at the beginning of the simulation. Checks that
    // contacts are detected before running the estimator.
    bool simStarted_ = false;

    // linear stiffness of contacts
    stateObservation::Matrix3 linStiffness_;
    // linear damping of contacts
    stateObservation::Matrix3 linDamping_;
    // angular stiffness of contacts
    stateObservation::Matrix3 angStiffness_;
    // linear damping of contacts
    stateObservation::Matrix3 angDamping_;

    // indicates if the debug logs have to be added.
    bool withDebugLogs_ = true;
    // indicates if we want to perform odometry along the stabilization or not.
    bool withOdometry_ = false;
    // associated to @withOdometry_. If true, the odometry on the position will be only along the x and y axes. If
    // false, the default 6D odometry is used.
    bool withFlatOdometry_ = false;

    // indicates what mode of contacts has to be used. \n Allowed modes are: \n fromSolver: The contacts are given by
    // the controller directly (then thresholded based on the measured force). \n fromSurfaces: The contacts are given
    // by the controller directly from a surface (then thresholded based on the measured force). \n fromThreshold: The
    // contacts are not required to be given by the controller (the detection is based on a thresholding of the measured
    // force)
    std::string contactsDetection_;
    // indicates if the forces measurement have to be filtered with a low-pass filter.
    bool withFilteredForcesContactDetection_ = false;

    // indicates if we want to estimate the unmodeled wrench within the Kinetics Observer.
    bool withUnmodeledWrench_ = true;
    // indicates if we want to estimate the bias on the gyrometer measurement within the Kinetics Observer.
    bool withGyroBias_ = true;

    // maximum amount of contacts that we want to use with the Kinetics Observer.
    int maxContacts_ = 4;
    // maximum amount of IMUs that we want to use with the Kinetics Observer.
    int maxIMUs_ = 2;

    stateObservation::Matrix3 statePositionInitCovariance_;
    stateObservation::Matrix3 stateOriInitCovariance_;
    stateObservation::Matrix3 stateLinVelInitCovariance_;
    stateObservation::Matrix3 stateAngVelInitCovariance_;
    stateObservation::Matrix3 gyroBiasInitCovariance_;
    stateObservation::Matrix6 unmodeledWrenchInitCovariance_;
    stateObservation::Matrix12 contactInitCovarianceFirstContacts_;
    stateObservation::Matrix12 contactInitCovarianceNewContacts_;

    stateObservation::Matrix3 statePositionProcessCovariance_;
    stateObservation::Matrix3 stateOriProcessCovariance_;
    stateObservation::Matrix3 stateLinVelProcessCovariance_;
    stateObservation::Matrix3 stateAngVelProcessCovariance_;
    stateObservation::Matrix3 gyroBiasProcessCovariance_;
    stateObservation::Matrix6 unmodeledWrenchProcessCovariance_;
    stateObservation::Matrix12 contactProcessCovariance_;

    stateObservation::Matrix3 positionSensorCovariance_;
    stateObservation::Matrix3 orientationSensorCoVariance_;
    stateObservation::Matrix3 acceleroSensorCovariance_;
    stateObservation::Matrix3 gyroSensorCovariance_;
    stateObservation::Matrix6 contactSensorCovariance_;

    /* Config variables */
  };

} // mc_state_observation
