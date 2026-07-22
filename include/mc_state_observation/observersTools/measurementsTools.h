/**
 * \file      measurementTools.h
 * \author    Arnaud Demont
 * \date       2023
 * \brief      Library for an easened handling of contacts and sensors in general.
 *
 * \details
 *
 *
 */

#pragma once

#include <mc_control/MCController.h>
#include <mc_observers/Observer.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rtc/log/Logger.h>
#include <mc_rtc/logging.h>

namespace mc_state_observation
{
namespace measurements
{

/**
 * Sensors manager for observers implemented in mc_rtc.
 **/

///////////////////////////////////////////////////////////////////////
/// -----------------------------Sensors-------------------------------
///////////////////////////////////////////////////////////////////////

/// @brief Class containing the information of a sensor to facilitate its handling.
struct Sensor
{
public:
  inline const int & getID() const { return id_; }
  inline const std::string & getName() const { return name_; }

protected:
  Sensor() {}
  ~Sensor() {}
  Sensor(int id, std::string name) : id_(id), name_(name) {}

  bool operator<(const Sensor & contact2) const { return (getID() < contact2.id_); }

protected:
  int id_;
  std::string name_;
};
///////////////////////////////////////////////////////////////////////
/// --------------------------------IMUs-------------------------------
///////////////////////////////////////////////////////////////////////

/// @brief Class containing the information of an IMU.
struct IMU : public Sensor
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
  Eigen::Vector3d gyroBias;
};

/// @brief List of IMUs.
/// @details Facilitates the handling of IMUs.
struct MapIMUs
{
public:
  /// @brief Get the index of the IMU given its name.
  /// @param name The name of the IMU
  /// @return const int &
  inline const int & getNumFromName(const std::string & name) { return mapIMUs_.find(name)->second.getID(); }
  /// @brief Get the name of the IMU given its index.
  /// @param num_ The index of the IMU
  /// @return const std::string &
  inline const std::string & getNameFromNum(const int & num) { return insertOrder_.at(num); }

  /// @brief Get the list of all the IMUs.
  /// @return const std::vector<std::string> &
  inline const std::vector<std::string> & getList() { return insertOrder_; }

  /// @brief Checks if the required IMU exists.
  /// @param name The name of the IMU
  /// @return bool
  inline bool hasElement(const std::string & name) { return checkAlreadyExists(name); }

  /// @brief Inserts an IMU to the map.
  /// @param name The name of the IMU
  inline void insertIMU(std::string name)
  {
    if(checkAlreadyExists(name)) return;
    insertOrder_.push_back(name);
    mapIMUs_.insert(std::make_pair(name, IMU(num_, name)));
    num_++;
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
  inline bool checkAlreadyExists(const std::string & name) { return mapIMUs_.find(name) != mapIMUs_.end(); }

private:
  // list of all the IMUs.
  std::vector<std::string> insertOrder_;
  // map associating all the IMUs to their names.
  std::unordered_map<std::string, IMU> mapIMUs_;
  // Index generator, incremented everytime a new IMU is added
  int num_ = 0;
};

///////////////////////////////////////////////////////////////////////
/// ------------------------------Contacts-----------------------------
///////////////////////////////////////////////////////////////////////

/**
 * Contacts manager for observers implemented in mc_rtc.
 * This contact manager handles the detection of contacts with three different methods, using contact surfaces, contacts
 * directly given by the controller, or a thresholding on the measured contact force.
 * On each iteration, the manager updates the list of current contacts and of removed contacts.
 **/

/* Contains the important variables associated to the contact */

struct Contact : public Sensor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  Contact() {}
  ~Contact() {}
  // constructor if the contact is not associated to a surface
  Contact(int id, std::string name)
  {
    id_ = id;
    name_ = name;
    resetContact();
  }
  // constructor if the contact is associated to a surface
  Contact(int id, std::string name, std::string surface) : Contact(id, name) { surfaceName(surface); }

public:
  inline void resetContact()
  {
    wasAlreadySet_ = false;
    isSet_ = false;
  }

  void surfaceName(std::string surfaceName) { surface_ = surfaceName; }
  const std::string & surfaceName() const
  {
    BOOST_ASSERT(!surface_.empty() && "The contact was created without a surface.");
    return surface_;
  }

  /*// ! Not working yet
  inline const Eigen::Vector3d & getZMP()
  {
    return zmp;
  }
  */

public:
  bool isSet_ = false;
  bool wasAlreadySet_ = false;
  // Eigen::Vector3d zmp; // ! Not working yet
protected:
  std::string surface_;
};

/**
 * Structure of contacts with sensors, containing all the necessary information about the contact. If the contact is
 *detected using a thresholding on the contact force, the contact force cannot be obtained and the name of the contact
 *will be the one of the force sensor. Otherwise the name of the contact surface is used, allowing the creation of
 *contacts associated to a same sensor but a different surface.
 **/
struct ContactWithSensor : public Contact
{

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  ContactWithSensor() {}
  // constructor if the contact is not associated to a surface
  ContactWithSensor(int id, std::string forceSensorName)
  {
    id_ = id;
    name_ = forceSensorName;
    forceSensorName_ = forceSensorName;

    resetContact();
  }

  // constructor if the contact is associated to a surface
  ContactWithSensor(int id,
                    const std::string & forceSensorName,
                    const std::string & surfaceName,
                    bool sensorAttachedToSurface)
  {
    id_ = id;
    name_ = surfaceName;
    resetContact();

    surface_ = surfaceName;
    forceSensorName_ = forceSensorName;
    sensorAttachedToSurface_ = sensorAttachedToSurface;
  }
  ~ContactWithSensor() {}
  inline void resetContact()
  {
    wasAlreadySet_ = false;
    isSet_ = false;
    sensorWasEnabled_ = false;

    // also filtered force? see when this feature will be corrected
  }

  std::string & forceSensorName() { return forceSensorName_; }

public:
  Eigen::Matrix<double, 6, 1> wrenchInCentroid_ = Eigen::Matrix<double, 6, 1>::Zero(); // for debug only
  double forceNorm_ = 0.0; // for debug only
  // the sensor measurement have to be used by the observer
  bool sensorEnabled_ = true;
  // allows to know if the contact's measurements have to be added during the update.
  bool sensorWasEnabled_ = false;

  // measured contact wrench, expressed in the frame of the contact. Not automatically computed so must be explicitely
  // computed and called.
  Eigen::Matrix<double, 6, 1> contactWrenchVector_;

  // indicates if the sensor is directly attached to a surface (true) or not (false). Default is true because in the
  // case of detection of contacts by thresholding the measured force (@contactsDetection_ = fromThreshold), we cannot
  // know precisely the surface of contact, so we will consider that the kinematics of the contact surface are the
  // ones of the sensor
  bool sensorAttachedToSurface_ = true;
  // surface of contact
protected:
  std::string forceSensorName_;

  /* Force filtering for the contact detection */ // ! Not working yet!
  // Eigen::Vector3d filteredForce = Eigen::Vector3d::Zero();
  // double lambda = 0.0;
};

/**
 * Structure of contacts without sensors, containing all the necessary information about the contact.
 **/
struct ContactWithoutSensor : public Contact
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  ContactWithoutSensor() {}
  ~ContactWithoutSensor() {}
  ContactWithoutSensor(int id, std::string name)
  {
    id_ = id;
    name_ = name;
    surfaceName(name);
    resetContact();
  }
};

/// @brief Map of contacts containing the list of all the contacts and functions facilitating their handling.
/// @details The template allows to define other kinds of contacts and thus add custom parameters to them. Warning! This
/// class has been tested only on contacts with sensors
/// @tparam ContactWithSensorT Contacts associated to a sensor.
/// @tparam ContactWithoutSensorT Contacts that are not associated to a sensor.
template<typename ContactWithSensorT, typename ContactWithoutSensorT>
struct MapContacts
{
public:
  MapContacts()
  {
    BOOST_ASSERT((std::is_base_of<ContactWithSensor, ContactWithSensorT>::value)
                 && "The template class for the contacts with sensors must inherit from the ContactWithSensor class");
    BOOST_ASSERT(
        (std::is_base_of<ContactWithoutSensor, ContactWithoutSensorT>::value)
        && "The template class for the contacts with sensors must inherit from the ContactWithoutSensor class");
  }

public:
  /// @brief Accessor for the a contact associated to a sensor contained in the map
  ///
  /// @param name The name of the contact to access
  /// @return contactsWithSensorT&
  inline ContactWithSensorT & contactWithSensor(const std::string & name)
  {
    BOOST_ASSERT(checkAlreadyExists(name, true) && "The requested sensor doesn't exist");
    return mapContactsWithSensors_.at(name);
  }
  /// @brief Accessor for the a contact associated to a sensor contained in the map
  ///
  /// @param num_ The index of the contact to access
  /// @return ContactWithSensor&
  inline ContactWithSensorT & contactWithSensor(const int & num)
  {
    BOOST_ASSERT((num >= 0 && num < num_) && "The requested sensor doesn't exist");
    BOOST_ASSERT(checkAlreadyExists(getNameFromNum(num), true) && "The requested sensor doesn't exist");
    return mapContactsWithSensors_.at(getNameFromNum(num));
  }

  /// @brief Accessor for the a contact that is not associated to a sensor contained in the map
  /// @param name The name of the contact to access
  /// @return ContactWithoutSensor&
  inline ContactWithoutSensorT & contactWithoutSensor(const std::string & name)
  {
    BOOST_ASSERT(checkAlreadyExists(name, false) && "The requested sensor doesn't exist");
    return mapContactsWithoutSensors_.at(name);
  }

  /// @brief Accessor for the a contact that is not associated to a sensor contained in the map
  /// @param num_ The index of the contact to access
  /// @return ContactWithoutSensorT&
  inline ContactWithoutSensorT & contactWithoutSensor(const int & num)
  {
    BOOST_ASSERT((num >= 0 && num < num_) && "The requested sensor doesn't exist");
    BOOST_ASSERT(checkAlreadyExists(getNameFromNum(num), true) && "The requested sensor doesn't exist");
    return mapContactsWithoutSensors_.at(getNameFromNum(num));
  }

  /// @brief Get the map of all the contacts associated to a sensor
  ///
  /// @return std::unordered_map<std::string, contactsWithSensorT>&
  inline std::unordered_map<std::string, ContactWithSensorT> & contactsWithSensors() { return mapContactsWithSensors_; }
  /// @brief Get the map of all the contacts that are not associated to a sensor
  ///
  /// @return std::unordered_map<std::string, ContactWithoutSensorT>&
  inline std::unordered_map<std::string, ContactWithoutSensorT> & contactsWithoutSensors()
  {
    return mapContactsWithoutSensors_;
  }

  /// @brief Get the list of all the contacts (with and without sensors)
  ///
  /// @return const std::vector<std::string> &
  inline const std::vector<std::string> & getList() { return insertOrder_; }

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
  /// @param num_ The index of the contact
  /// @return const std::string &
  inline const std::string & getNameFromNum(const int & num) { return insertOrder_.at(num); }

  /// @brief Get the index of a contact given its name
  ///
  /// @param name The name of the contact
  /// @return const int &
  inline const int & getNumFromName(const std::string & name)
  {
    if(hasSensor_.at(name)) { return mapContactsWithSensors_.at(name).getID(); }
    else { return mapContactsWithoutSensors_.at(name).getID(); }
  }

  /* // ! Not working yet
  /// @brief Get the measured zmp of a contact given its name
  ///
  /// @param name The name of the contact
  /// @return const Eigen::Vector3d &
  inline const Eigen::Vector3d & getZMPFromName(const std::string & name)
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
  */

  /// @brief Checks if the given contact exists
  ///
  /// @param element The name of the contact
  /// @return bool
  inline bool hasElement(const std::string & element) { return hasSensor_.find(element) != hasSensor_.end(); }

  /// @brief Check that a contact still does not exist, if so, insert a contact to the map of contacts. The contact
  /// can either be associated to a sensor or not.
  /// @details Version for contacts that are either associated to a surface or to a force sensor.
  /// @param element The name of the contact. If the contact has a sensor, its name is the one of the sensor, else, its
  /// name is the one of the surface.
  /// @param hasSensor True if the contact is attached to a sensor.
  inline void insertContact(const std::string & name, const bool & hasSensor)
  {
    if(checkAlreadyExists(name, hasSensor)) return;
    insertElement(name, hasSensor);

    num_++;
  }

  /// @brief Check that a contact still does not exist, if so, insert a contact to the map of contacts.
  /// @details Version for contacts that are associated to both a force sensor and a contact surface. The contact will
  /// be named with the name of the surface.
  /// @param forceSensorName The name of the force sensor.
  /// @param surface The name of the surface that will be used also to name the contact.
  /// @param sensorAttachedToSurface True if the sensor is attached to the surface.
  inline void insertContact(const std::string & forceSensorName,
                            const std::string surface,
                            const bool & sensorAttachedToSurface)
  {
    if(checkAlreadyExists(sensorAttachedToSurface, surface)) return;
    insertElement(forceSensorName, surface, sensorAttachedToSurface);

    num_++;
  }

private:
  /// @brief Insert a contact to the map of contacts. The contact can either be associated to a sensor or not.
  /// @details Version for contacts that are associated to both a force sensor and a contact surface. The contact will
  /// be named with the name of the surface.
  /// @param forceSensorName The name of the force sensor.
  /// @param surface The name of the surface that will be used also to name the contact.
  /// @param sensorAttachedToSurface True if the sensor is attached to the surface.
  inline void insertElement(const std::string & forceSensorName,
                            const std::string surface,
                            const bool sensorAttachedToSurface)
  {
    insertOrder_.push_back(surface);

    mapContactsWithSensors_.insert(
        std::make_pair(surface, ContactWithSensorT(num_, forceSensorName, surface, sensorAttachedToSurface)));
    hasSensor_.insert(std::make_pair(surface, true));
  }
  /// @brief Insert a contact to the map of contacts. The contact can either be associated to a sensor or not.
  /// @details Version for contacts that are either associated to a surface or to a force sensor.
  /// @param name The name of the contact. If hasSensor is true, the name is the one of the forceSensor, else, the name
  /// is the one of the surface.
  /// @param hasSensor True if the contact is attached to a sensor.
  inline void insertElement(const std::string & name, const bool & hasSensor)
  {
    insertOrder_.push_back(name);

    if(hasSensor)
    {
      mapContactsWithSensors_.insert(std::make_pair(name, ContactWithSensorT(num_, name)));
      hasSensor_.insert(std::make_pair(name, true));
    }
    else
    {
      mapContactsWithoutSensors_.insert(std::make_pair(name, ContactWithoutSensorT(num_, name)));
      hasSensor_.insert(std::make_pair(name, true));
    }
  }

  /// @brief Check if a contact already exists in the list. If it already exists, checks that the contact remained
  /// unchanged.
  ///
  /// @param name The name of the contact
  /// @param hasSensor True if the contact is attached to a sensor.
  /// @return bool
  inline bool checkAlreadyExists(const std::string & name, const bool hasSensor)
  {
    if(std::find(insertOrder_.begin(), insertOrder_.end(), name) != insertOrder_.end()) // the contact already exists
    {
      BOOST_ASSERT_MSG(hasSensor_.at(name) == hasSensor,
                       "The association / non-association to a force sensor must be preserved.");

      return true;
    }
    else { return false; }
  }

  /// @brief Check if a contact already exists in the list. If it already exists, checks that the contact remained
  /// unchanged.
  ///
  /// @param name The name of the contact
  /// @param hasSensor True if the contact is attached to a sensor.
  /// @param sensorAttachedToSurface True if the sensor is attached to the surface.
  /// @return bool
  inline bool checkAlreadyExists(bool sensorAttachedToSurface, const std::string & name)
  {
    if(std::find(insertOrder_.begin(), insertOrder_.end(), name) != insertOrder_.end()) // the contact already exists
    {
      if(!hasSensor_.at(name))
      {
        mc_rtc::log::error_and_throw("The contact already exists and was associated to no sensor");
      }
      if(contactWithSensor(name).sensorAttachedToSurface_ != sensorAttachedToSurface)
      {
        mc_rtc::log::error_and_throw(
            "You previously said that the contact sensor was not attached to the contact surface");
      }
      return true;
    }
    else { return false; }
  }

private:
  // map containing all the contacts and indicating if each sensor has a contact or not
  std::unordered_map<std::string, bool> hasSensor_;
  // map containing all the contacts associated to a sensor
  std::unordered_map<std::string, ContactWithSensorT> mapContactsWithSensors_;
  // map containing all the contacts that are not associated to a sensor
  std::unordered_map<std::string, ContactWithoutSensorT> mapContactsWithoutSensors_;
  // List of all the contacts
  std::vector<std::string> insertOrder_;
  // Index generator, incremented everytime a new contact is created
  int num_ = 0;
};

/// @brief Structure that implements all the necessary functions to manage the map of contacts. Handles their detection
/// and updates the list of the detected contacts, newly removed contacts, etc., to apply the appropriate functions on
/// them.
/// @details The template allows to define other kinds of contacts and thus add custom parameters to them. Warning! This
/// class has been tested only on contacts with sensors
/// @tparam ContactWithSensorT Contacts associated to a sensor.
/// @tparam ContactWithoutSensorT Contacts that are not associated to a sensor.
template<typename ContactWithSensorT, typename ContactWithoutSensorT>
struct ContactsManager
{
public:
  enum ContactsDetection
  {
    fromSolver,
    fromSurfaces,
    fromThreshold,
    undefined
  };

public:
  ContactsManager()
  {
    BOOST_ASSERT((std::is_base_of<ContactWithSensor, ContactWithSensorT>::value)
                 && "The template class for the contacts with sensors must inherit from the ContactWithSensor class");
    BOOST_ASSERT((std::is_base_of<ContactWithoutSensor, ContactWithoutSensorT>::value)
                 && "The template class for the contacts with sensors must inherit from the ContactWithSensor class");
  }
  ~ContactsManager() {}

  // initialization of the odometry
  void init(const std::string & observerName, const bool verbose = true);

  /// @brief Initialization for a detection based on contact surfaces
  /// @param ctl Controller
  /// @param robotName name of the robot
  /// @param contactsDetection mean of detection for the contacts
  /// @param surfacesForContactDetection list of possible contact surfaces
  /// @param contactsSensorDisabledInit list of the force sensors that must be disabled on initialization.
  /// @param contactDetectionThreshold threshold on the measured force for the contact detection
  void initDetection(const mc_control::MCController & ctl,
                     const std::string & robotName,
                     const ContactsDetection & contactsDetection,
                     const std::vector<std::string> & surfacesForContactDetection,
                     const std::vector<std::string> & contactsSensorDisabledInit,
                     const double & contactDetectionThreshold);

  /// @brief initialization for a detection based on a threshold on the measured contact forces or for contacts given by
  /// the controller
  /// @param ctl Controller
  /// @param robotName name of the robot
  /// @param contactsDetection mean of detection for the contacts
  /// @param contactsSensorDisabledInit list of the force sensors that must be disabled on initialization.
  /// @param contactDetectionThreshold threshold on the measured force for the contact detection
  /// @param forceSensorsToOmit list of force sensors that cannot be used for the contacts detection
  void initDetection(const mc_control::MCController & ctl,
                     const std::string & robotName,
                     const ContactsDetection & contactsDetection,
                     const std::vector<std::string> & contactsSensorDisabledInit,
                     const double & contactDetectionThreshold,
                     const std::vector<std::string> & forceSensorsToOmit);

  /// @brief Adds the contact to the GUI to enable or disable it easily.
  /// @details Version for a contact associated to a force sensor.
  /// @param ctl The controller.
  /// @param sensorName Sensor attached to the contact.
  void addContactToGui(const mc_control::MCController & ctl, const std::string & sensorName);

  /// @brief Adds the contact to the GUI to enable or disable it easily.
  /// @details Version for a contact associated to a surface.
  /// @param ctl The controller.
  /// @param surface Surface of the contact.
  /// @param forSurfaceContact Indicates that the contact is attached to a surface. Allows to differentiate from
  /// addContactToGui(const mc_control::MCController &, const std::string &).
  void addContactToGui(const mc_control::MCController & ctl, const std::string & surface, bool forSurfaceContact);

  typedef std::set<int> ContactsSet;

  std::string set_to_string(const ContactsSet & contactSet);

  /// @brief Updates the list of currently set contacts and returns it.
  /// @return std::set<FoundContactsListType> &
  const ContactsSet & findContacts(const mc_control::MCController & ctl, const std::string & robotName);
  /// @brief Updates the list @contactsFound_ of currently set contacts directly from the controller.
  /// @details Called by \ref findContacts(const mc_control::MCController & ctl) if @contactsDetection_ is equal to
  /// "fromSolver". The contacts are given by the controller directly (then thresholded based on the measured force).
  void findContactsFromSolver(const mc_control::MCController & ctl, const std::string & robotName);
  /// @brief Updates the list @contactsFound_ of currently set contacts from the surfaces given by the user.
  /// @details Called by \ref findContacts(const mc_control::MCController & ctl) if @contactsDetection_ is equal to
  /// "fromSurfaces". The contacts are obtained by thresholded based the force measured by the associated force sensor).
  void findContactsFromSurfaces(const mc_control::MCController & ctl, const std::string & robotName);
  /// @brief Updates the list @contactsFound_ of currently set contacts from a thresholding of the measured forces.
  /// @details Called by \ref findContacts(const mc_control::MCController & ctl) if @contactsDetection_ is equal to
  /// "fromThreshold". The contacts are not required to be given by the controller (the detection is based on a
  /// thresholding of the measured force).
  void findContactsFromThreshold(const mc_control::MCController & ctl, const std::string & robotName);
  /// @brief Updates the list of contacts to inform whether they are newly set, removed, etc.
  void updateContacts();

  void (ContactsManager::*contactsFinder_)(const mc_control::MCController &, const std::string &) = 0;

  /// @brief Accessor for the a contact associated to a sensor contained in the map
  ///
  /// @param name The name of the contact to access
  /// @return contactsWithSensorT&
  inline ContactWithSensorT & contactWithSensor(const std::string & name)
  {
    return mapContacts_.contactWithSensor(name);
  }

  /// @brief Accessor for the a contact associated to a sensor contained in the map
  ///
  /// @param num The index of the contact to access
  /// @return ContactWithSensor&
  inline ContactWithSensorT & contactWithSensor(const int & num) { return mapContacts_.contactWithSensor(num); }

  /// @brief Accessor for the a contact associated to a sensor contained in the map
  ///
  /// @param num The contact itself.
  /// @return ContactWithSensor&
  inline ContactWithSensorT & contactWithSensor(ContactWithSensorT & contact) { return contact; }

  /// @brief Accessor for the a contact that is not associated to a sensor contained in the map
  /// @param name The name of the contact to access
  /// @return ContactWithoutSensor&
  inline ContactWithoutSensorT & contactWithoutSensor(const std::string & name)
  {
    return mapContacts_.contactWithoutSensor(name);
  }

  /// @brief Accessor for the a contact that is not associated to a sensor contained in the map
  /// @param num_ The index of the contact to access
  /// @return ContactWithoutSensorT&
  inline ContactWithoutSensorT & contactWithoutSensor(const int & num)
  {
    return mapContacts_.contactWithoutSensor(num);
  }

  /// @brief Accessor for the a contact that is not associated to a sensor contained in the map
  /// @param name The contact itself.
  /// @return ContactWithoutSensor&
  inline ContactWithoutSensorT & contactWithoutSensor(const ContactWithoutSensorT & contact) { return contact; }

  /// @brief Get the map of all the contacts associated to a sensor
  ///
  /// @return std::unordered_map<std::string, contactsWithSensorT>&
  inline std::unordered_map<std::string, ContactWithSensorT> & contactsWithSensors()
  {
    return mapContacts_.contactsWithSensors();
  }
  /// @brief Get the map of all the contacts that are not associated to a sensor
  ///
  /// @return std::unordered_map<std::string, ContactWithoutSensorT>&
  inline std::unordered_map<std::string, ContactWithoutSensorT> & contactsWithoutSensors()
  {
    return mapContacts_.contactsWithoutSensors();
  }

  /// @brief Get the list of all the contacts.
  /// @return const std::vector<std::string> &
  inline const std::vector<std::string> & getList() { return mapContacts_.getList(); }

  /// @brief Get the list of the currently set contacts.
  /// @return const std::vector<std::string> &
  inline const ContactsSet & contactsFound() { return contactsFound_; }

  inline const ContactsSet & removedContacts() { return removedContacts_; }

  inline const ContactsDetection & getContactsDetection() { return contactsDetectionMethod_; }

public:
  // map of contacts used by the manager.
  MapContacts<ContactWithSensorT, ContactWithoutSensorT> mapContacts_;

protected:
  double contactDetectionThreshold_;
  // list of the currently set contacts. The custom comparator is used to ensure that the sorting of contacts is
  // consistent

  // list of the current contacts
  ContactsSet contactsFound_;

  // list of contacts that were set on last iteration
  ContactsSet oldContacts_;
  // list of the contacts that just got removed
  ContactsSet removedContacts_;

  // list of surfaces used for contacts detection if @contactsDetection_ is set to "fromSurfaces"
  std::vector<std::string> surfacesForContactDetection_;
  // list of sensors that must not be used from the start of the observer
  std::vector<std::string> contactsSensorDisabledInit_;
  // name of the observer using this contacts manager.
  std::string observerName_;
  // method used to detect the contacts
  ContactsDetection contactsDetectionMethod_ = undefined;
  bool verbose_ = true;
};

// allowed odometry types
enum OdometryType
{
  odometry6d,
  flatOdometry,
  None
};

} // namespace measurements
} // namespace mc_state_observation

#include <mc_state_observation/observersTools/measurementsTools.hpp>
