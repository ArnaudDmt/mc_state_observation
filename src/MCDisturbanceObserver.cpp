#include <mc_state_observation/MCDisturbanceObserver.h>

#include <mc_observers/ObserverMacros.h>
#include <mc_rbdyn/BodySensor.h>
#include <mc_rbdyn/ForceSensor.h>
#include <mc_rtc/logging.h>

namespace mc_state_observation
{

MCDisturbanceObserver::MCDisturbanceObserver(const std::string & type, double dt) : mc_observers::Observer(type, dt) {}

void MCDisturbanceObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());

  const auto & robot = ctl.robot(robot_);

  imuName_ = config("imu", robot.bodySensor().name());
  nominalMass_ = config("nominalMass", ctl.realRobot(robot_).mass());
  cutoffFrequency_ = config("cutoffFrequency", 100.0);

  listIMUs_.clear();
  listIMUs_.push_back({0, imuName_});

  auto contactsConfig = config("contacts");

  std::string contactsDetectionString = static_cast<std::string>(contactsConfig("contactsDetection"));
  DoContactsDetector::ContactsDetection contactsDetectionMethod =
      DoContactsDetector::stringToContactsDetection(contactsDetectionString, name());

  if(contactsDetectionMethod == DoContactsDetector::ContactsDetection::Surfaces)
  {
    std::vector<std::string> surfacesForContactDetection =
        contactsConfig("surfacesForContactDetection", std::vector<std::string>());

    for(const auto & surface : surfacesForContactDetection)
    {
      const std::string fsName = ctl.robot(robot_).indirectSurfaceForceSensor(surface).name();
      contactsManager_.fs_Surface_Map.emplace(fsName, surface);
    }

    measurements::ContactsDetectorSurfacesConfiguration contactsConf(surfacesForContactDetection);

    if(contactsConfig.has("schmittTriggerLowerPropThreshold") && contactsConfig.has("schmittTriggerUpperPropThreshold"))
    {
      double schmittTriggerLowerPropThreshold = contactsConfig("schmittTriggerLowerPropThreshold");
      double schmittTriggerUpperPropThreshold = contactsConfig("schmittTriggerUpperPropThreshold");
      contactsConf.schmittTriggerPropThresholds(schmittTriggerLowerPropThreshold, schmittTriggerUpperPropThreshold);
    }

    contactsDetector_.init(ctl, robot_, contactsConf);
  }

  if(contactsDetectionMethod == DoContactsDetector::ContactsDetection::Sensors)
  {
    measurements::ContactsDetectorSensorsConfiguration contactsConf;

    if(contactsConfig.has("schmittTriggerLowerPropThreshold") && contactsConfig.has("schmittTriggerUpperPropThreshold"))
    {
      double schmittTriggerLowerPropThreshold = contactsConfig("schmittTriggerLowerPropThreshold");
      double schmittTriggerUpperPropThreshold = contactsConfig("schmittTriggerUpperPropThreshold");
      contactsConf.schmittTriggerPropThresholds(schmittTriggerLowerPropThreshold, schmittTriggerUpperPropThreshold);
    }

    contactsDetector_.init(ctl, robot_, contactsConf);
  }

  if(contactsDetectionMethod == DoContactsDetector::ContactsDetection::Solver)
  {
    measurements::ContactsDetectorSolverConfiguration contactsConf;

    if(contactsConfig.has("schmittTriggerLowerPropThreshold") && contactsConfig.has("schmittTriggerUpperPropThreshold"))
    {
      double schmittTriggerLowerPropThreshold = contactsConfig("schmittTriggerLowerPropThreshold");
      double schmittTriggerUpperPropThreshold = contactsConfig("schmittTriggerUpperPropThreshold");
      contactsConf.schmittTriggerPropThresholds(schmittTriggerLowerPropThreshold, schmittTriggerUpperPropThreshold);
    }

    contactsDetector_.init(ctl, robot_, contactsConf);
  }

  mc_rtc::log::info("[{}] robot={} imu={} nominalMass={} cutoffFrequency={}", name(), robot_, imuName_, nominalMass_,
                    cutoffFrequency_);
}

void MCDisturbanceObserver::reset(const mc_control::MCController &)
{
  estimatedExternalForceRaw_.setZero();
  estimatedExternalForce_.setZero();
  previousEstimatedExternalForce_.setZero();

  for(auto & [_, contact] : contactsManager_.contacts()) { contact.resetContact(); }
}

bool MCDisturbanceObserver::run(const mc_control::MCController & ctl)
{
  const auto & realRobot = ctl.realRobot(robot_);

  updateContacts(ctl);
  computeEstimatedForce(ctl, realRobot);
  applyFirstOrderLowPassFilter();

  return true;
}

void MCDisturbanceObserver::update(mc_control::MCController &) {}

void MCDisturbanceObserver::updateContacts(const mc_control::MCController & ctl)
{
  std::unordered_set<std::string> & contactList = contactsDetector_.updateContacts(ctl, robot_);

  for(auto & [surfaceName, contact] : contactsManager_.contacts())
  {
    if(contactList.find(surfaceName) == contactList.end()) { contact.resetContact(); }
  }

  for(const auto & surfaceName : contactList)
  {
    auto * contact = contactsManager_.findContact(surfaceName);

    if(!contact)
    {
      auto & newContact =
          contactsManager_.contacts().emplace(surfaceName, DoContactWithSensor(surfaceName)).first->second;
      newContact.fsName(ctl.robot(robot_).indirectSurfaceForceSensor(surfaceName).name());
      newContact.sensorEnabled_ = true;
      newContact.contactId(0);
      newContact.setContact();
    }
    else
    {
      if(contact->fsName().empty())
      {
        contact->fsName(ctl.robot(robot_).indirectSurfaceForceSensor(surfaceName).name());
      }
      contact->sensorEnabled_ = true;
      contact->setContact();
    }
  }
}

Eigen::Matrix3d MCDisturbanceObserver::worldRotationOfIMU(const mc_rbdyn::Robot & robot) const
{
  const auto & imu = robot.bodySensor(imuName_);
  const auto bodyIndex = robot.bodyIndexByName(imu.parentBody());
  const sva::PTransformd X_0_b = robot.mbc().bodyPosW[bodyIndex];
  const sva::PTransformd X_b_s = imu.X_b_s();
  const sva::PTransformd X_0_s = X_0_b * X_b_s;
  return X_0_s.rotation().transpose();
}

void MCDisturbanceObserver::computeEstimatedForce(const mc_control::MCController & ctl,
                                                  const mc_rbdyn::Robot & realRobot)
{
  const auto & imu = realRobot.bodySensor(imuName_);

  const Eigen::Matrix3d R_0_is = worldRotationOfIMU(realRobot);
  const Eigen::Vector3d alphaWorld = R_0_is * imu.linearAcceleration();

  Eigen::Vector3d summedContactForce = Eigen::Vector3d::Zero();

  for(const auto & [surfaceName, contact] : contactsManager_.contacts())
  {
    if(!contact.isSet()) { continue; }
    if(!contact.sensorEnabled_) { continue; }

    const auto & fs = realRobot.forceSensor(contact.fsName());
    summedContactForce += fs.worldWrenchWithoutGravity(realRobot).force();
  }

  estimatedExternalForceRaw_ = nominalMass_ * alphaWorld - summedContactForce;
}

void MCDisturbanceObserver::applyFirstOrderLowPassFilter()
{
  const double alpha = std::exp(-cutoffFrequency_ * dt_);
  estimatedExternalForce_ = alpha * previousEstimatedExternalForce_ + (1.0 - alpha) * estimatedExternalForceRaw_;
  previousEstimatedExternalForce_ = estimatedExternalForce_;
}

void MCDisturbanceObserver::addToLogger(const mc_control::MCController &,
                                        mc_rtc::Logger & logger,
                                        const std::string & category)
{
  category_ = category;

  logger.addLogEntry(category_ + "_estimatedExternalForce", this, [this]() { return estimatedExternalForce_; });
  logger.addLogEntry(category_ + "_estimatedExternalForceRaw", this, [this]() { return estimatedExternalForceRaw_; });
}

void MCDisturbanceObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_estimatedExternalForce");
  logger.removeLogEntry(category + "_estimatedExternalForceRaw");
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("MCDisturbanceObserver", mc_state_observation::MCDisturbanceObserver)