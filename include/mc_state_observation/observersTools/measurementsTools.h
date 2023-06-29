#pragma once

#include <mc_control/MCController.h>
#include <mc_observers/Observer.h>
#include <mc_rbdyn/Robot.h>

namespace mc_state_observation
{
namespace measurements
{

///////////////////////////////////////////////////////////////////////
/// -----------------------------Sensors-------------------------------
///////////////////////////////////////////////////////////////////////
struct Sensor
{
public:
  inline const int & getID() const
  {
    return id_;
  }
  inline const std::string & getName() const
  {
    return name_;
  }

protected:
  Sensor() {}
  ~Sensor() {}
  Sensor(int id, std::string name) : id_(id), name_(name) {}

  bool operator<(const Sensor & contact2) const
  {
    return (getID() < contact2.id_);
  }

protected:
  int id_;
  std::string name_;
};
///////////////////////////////////////////////////////////////////////
/// --------------------------------IMUs-------------------------------
///////////////////////////////////////////////////////////////////////

/* Contains the important variables associated to the IMU */
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
  /// @param num_ The index of the IMU
  /// @return const std::string &
  inline const std::string & getNameFromNum(const int & num)
  {
    return insertOrder_.at(num);
  }

  /// @brief Get the list of all the IMUs.
  /// @return const std::vector<std::string> &
  inline const std::vector<std::string> & getList()
  {
    return insertOrder_;
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
  inline bool checkAlreadyExists(const std::string & name)
  {
    return mapIMUs_.find(name) != mapIMUs_.end();
  }

private:
  // list of all the IMUs.
  std::vector<std::string> insertOrder_;
  // map associating all the IMUs to their names.
  std::map<std::string, IMU> mapIMUs_;
  // Index generator, incremented everytime a new IMU is added
  int num_ = 0;
};

///////////////////////////////////////////////////////////////////////
/// ------------------------------Contacts-----------------------------
///////////////////////////////////////////////////////////////////////

/* Contains the important variables associated to the contact */

struct Contact : public Sensor
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
    wasAlreadySet_ = false;
    isSet_ = false;
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
};

struct ContactWithSensor : public Contact
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
public:
  ContactWithSensor() {}
  ContactWithSensor(int id, std::string name)
  {
    id_ = id;
    name_ = name;
    resetContact();
  }
  ~ContactWithSensor() {}
  inline void resetContact()
  {
    wasAlreadySet_ = false;
    isSet_ = false;
    sensorWasEnabled = false;

    // also filtered force? see when this feature will be corrected
  }

public:
  Eigen::Matrix<double, 6, 1> wrenchInCentroid = Eigen::Matrix<double, 6, 1>::Zero(); // for debug only
  double forceNorm_ = 0.0; // for debug only
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

  /* Force filtering for the contact detection */ // ! Not working yet!
  // Eigen::Vector3d filteredForce = Eigen::Vector3d::Zero();
  // double lambda = 0.0;
};

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
    resetContact();
  }
};

template<typename ContactWithSensorT, typename ContactWithoutSensorT>
struct MapContacts
{
public:
  MapContacts()
  {
    BOOST_ASSERT((std::is_base_of<ContactWithSensor, ContactWithSensorT>::value)
                 && "The template class for the contacts with sensors must inherit from the ContactWithSensor class");
    BOOST_ASSERT((std::is_base_of<ContactWithoutSensor, ContactWithoutSensorT>::value)
                 && "The template class for the contacts with sensors must inherit from the ContactWithSensor class");
  }

public:
  /// @brief Accessor for the a contact associated to a sensor contained in the map
  ///
  /// @param name The name of the contact to access
  /// @return contactsWithSensorT&
  inline ContactWithSensorT & contactWithSensor(const std::string & name)
  {
    return mapContactsWithSensors_.at(name);
  }
  /// @brief Accessor for the a contact associated to a sensor contained in the map
  ///
  /// @param num_ The index of the contact to access
  /// @return ContactWithSensor&
  inline ContactWithSensorT & contactWithSensor(const int & num)
  {
    return mapContactsWithSensors_.at(getNameFromNum(num));
  }

  /// @brief Accessor for the a contact that is not associated to a sensor contained in the map
  /// @param name The name of the contact to access
  /// @return ContactWithoutSensor&
  inline ContactWithoutSensorT & contactWithoutSensor(const std::string & name)
  {
    return mapContactsWithoutSensors_.at(name);
  }

  /// @brief Accessor for the a contact that is not associated to a sensor contained in the map
  /// @param num_ The index of the contact to access
  /// @return ContactWithoutSensorT&
  inline ContactWithoutSensorT & contactWithoutSensor(const int & num)
  {
    return mapContactsWithoutSensors_.at(getNameFromNum(num));
  }

  /// @brief Get the map of all the contacts associated to a sensor
  ///
  /// @return std::map<std::string, contactsWithSensorT>&
  inline std::map<std::string, ContactWithSensorT> & contactsWithSensors()
  {
    return mapContactsWithSensors_;
  }
  /// @brief Get the map of all the contacts that are not associated to a sensor
  ///
  /// @return std::map<std::string, ContactWithoutSensorT>&
  inline std::map<std::string, ContactWithoutSensorT> & contactsWithoutSensors()
  {
    return mapContactsWithoutSensors_;
  }

  /// @brief Get the list of all the contacts (with and without sensors)
  ///
  /// @return const std::vector<std::string> &
  inline const std::vector<std::string> & getList()
  {
    return insertOrder_;
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
  /// @param num_ The index of the contact
  /// @return const std::string &
  inline const std::string & getNameFromNum(const int & num)
  {
    return insertOrder_.at(num);
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
    insertElement(name, hasSensor);

    num_++;
  }

  inline void insertContact(const std::string & name, const std::string surface, const bool & sensorAttachedToSurface)
  {
    if(checkAlreadyExists(name, surface, sensorAttachedToSurface)) return;
    insertElement(name, surface, sensorAttachedToSurface);

    num_++;
  }

private:
  inline void insertElement(const std::string & name, const std::string surface, const bool & sensorAttachedToSurface)
  {
    insertOrder_.push_back(name);

    mapContactsWithSensors_.insert(std::make_pair(name, ContactWithSensorT(num_, name)));
    contactWithSensor(name).surface = surface;
    contactWithSensor(name).sensorAttachedToSurface = sensorAttachedToSurface;
    hasSensor_.insert(std::make_pair(name, true));
  }
  /// @brief Insert a contact to the map of contacts. The contact can either be associated to a sensor or not.
  ///
  /// @param element The name of the contact
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

  inline bool checkAlreadyExists(const std::string & name,
                                 const std::string & surface,
                                 const bool & sensorAttachedToSurface)
  {
    if(std::binary_search(insertOrder_.begin(), insertOrder_.end(), name)) // the contact already exists
    {
      BOOST_ASSERT((!hasSensor(name)) && "The contact already exists and was associated to no sensor");
      BOOST_ASSERT((contactWithSensor(name).surface != surface)
                   && "The contact already exists but was associated to another surface");
      BOOST_ASSERT((contactWithSensor(name).sensorAttachedToSurface != sensorAttachedToSurface)
                   && "You previously said that the contact sensor was not attached to the contact surface");
      return true;
    }
    else
    {
      return false;
    }
  }

  /// @brief Check if a contact already exists in the list. If it already exists, checks that the association to a
  /// sensor remained unchanged.
  ///
  /// @param element The name of the contact
  /// @param hasSensor True if the contact is attached to a sensor.
  /// @return bool
  inline bool checkAlreadyExists(const std::string & name, const bool & hasSensor)
  {
    if(std::binary_search(insertOrder_.begin(), insertOrder_.end(), name)) // the contact already exists
    {
      BOOST_ASSERT((hasSensor(name) && !hasSensor) && "The contact already exists and was associated to a sensor");
      BOOST_ASSERT((!hasSensor(name) && hasSensor) && "The contact already exists and was associated to no sensor");

      return true;
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
  std::map<std::string, ContactWithSensorT> mapContactsWithSensors_;
  // map containing all the contacts that are not associated to a sensor
  std::map<std::string, ContactWithoutSensorT> mapContactsWithoutSensors_;
  // List of all the contacts
  std::vector<std::string> insertOrder_;
  // Index generator, incremented everytime a new contact is created
  int num_ = 0;
};

// the template allows to define other kinds of contacts and thus add custom parameters to them
// ! Warning: this class has been tested only on contacts with sensors
template<typename ContactWithSensorT, typename ContactWithoutSensorT>
struct ContactsManager
{

public:
  ContactsManager()
  {
    BOOST_ASSERT((std::is_base_of<ContactWithSensor, ContactWithSensorT>::value)
                 && "The template class for the contacts with sensors must inherit from the ContactWithSensor class");
    BOOST_ASSERT((std::is_base_of<ContactWithoutSensor, ContactWithoutSensorT>::value)
                 && "The template class for the contacts with sensors must inherit from the ContactWithSensor class");
  }
  ~ContactsManager() {}

  // initialization for a detection based on contact surfaces
  void init(const mc_control::MCController & ctl,
            const std::string & robotName,
            const std::string & observerName,
            const std::string contactsDetection,
            std::vector<std::string> surfacesForContactDetection,
            std::vector<std::string> contactsSensorDisabledInit,
            const double contactDetectionThreshold,
            const bool verbose = true);
  // initialization for a detection based on a threshold on the measured contact forces or for contacts given by the
  // controller
  void init(const mc_control::MCController & ctl,
            const std::string & robotName,
            const std::string & observerName,
            const std::string contactsDetection,
            std::vector<std::string> contactsSensorDisabledInit,
            const double contactDetectionThreshold,
            const bool verbose = true);

  typedef std::set<int> ContactsSet;
  std::string to_string(const ContactsSet & contactSet)
  {
    if(contactSet.cbegin() == contactSet.cend())
    {
      return "";
    }
    std::ostringstream out;
    out.precision(std::numeric_limits<int>::digits10);
    out << std::fixed << mapContacts_.getNameFromNum(*contactSet.cbegin());

    for(auto it = std::next(contactSet.cbegin()); it != contactSet.cend(); ++it)
    {
      out << ", ";
      out << std::fixed << mapContacts_.getNameFromNum(*it);
    }
    return out.str();
  }

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
  /// @brief Updates the detected and removed contacts.
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
  inline ContactWithSensorT & contactWithSensor(const int & num)
  {
    return mapContacts_.contactWithSensor(num);
  }

  /// @brief Accessor for the a contact associated to a sensor contained in the map
  ///
  /// @param num The contact itself.
  /// @return ContactWithSensor&
  inline ContactWithSensorT & contactWithSensor(ContactWithSensorT & contact)
  {
    return contact;
  }

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
  inline ContactWithoutSensorT & contactWithoutSensor(const ContactWithoutSensorT & contact)
  {
    return contact;
  }

  /// @brief Get the map of all the contacts associated to a sensor
  ///
  /// @return std::map<std::string, contactsWithSensorT>&
  inline std::map<std::string, ContactWithSensorT> & contactsWithSensors()
  {
    return mapContacts_.contactsWithSensors();
  }
  /// @brief Get the map of all the contacts that are not associated to a sensor
  ///
  /// @return std::map<std::string, ContactWithoutSensorT>&
  inline std::map<std::string, ContactWithoutSensorT> & contactsWithoutSensors()
  {
    return mapContacts_.contactsWithoutSensors();
  }

  /// @brief Get the list of all the contacts.
  /// @return const std::vector<std::string> &
  inline const std::vector<std::string> & getList()
  {
    return mapContacts_.getList();
  }

  /// @brief Get the list of the currently set contacts.
  /// @return const std::vector<std::string> &
  inline const ContactsSet & contactsFound()
  {
    return contactsFound_;
  }

  inline const ContactsSet & removedContacts()
  {
    return removedContacts_;
  }

public:
  MapContacts<ContactWithSensorT, ContactWithoutSensorT> mapContacts_;

  // contacts found on each iteration

protected:
  double contactDetectionThreshold_;
  // list of the currently set contacts. The custom comparator is used to ensure that the sorting of contacts is
  // consistent

  ContactsSet contactsFound_;
  ContactsSet oldContacts_;
  ContactsSet removedContacts_;

  // list of surfaces used for contacts detection if @contactsDetection_ is set to "fromSurfaces"
  std::vector<std::string> surfacesForContactDetection_;
  // list of sensors that must not be used from the start of the observer
  std::vector<std::string> contactsSensorDisabledInit_;
  std::string observerName_;
  bool verbose_ = true;
};

} // namespace measurements
} // namespace mc_state_observation

#include <mc_state_observation/observersTools/measurementsTools.hpp>