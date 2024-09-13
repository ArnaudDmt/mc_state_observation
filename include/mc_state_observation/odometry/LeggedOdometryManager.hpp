#pragma once
#include <mc_state_observation/conversions/kinematics.h>
#include <mc_state_observation/odometry/LeggedOdometryManager.h>

namespace mc_state_observation::odometry
{
template<typename OnNewContactObserver,
         typename OnMaintainedContactObserver,
         typename OnRemovedContactObserver,
         typename OnAddedContactObserver>
void LeggedOdometryManager::initLoop(const mc_control::MCController & ctl,
                                     mc_rtc::Logger & logger,
                                     const RunParameters<OnNewContactObserver,
                                                         OnMaintainedContactObserver,
                                                         OnRemovedContactObserver,
                                                         OnAddedContactObserver> & runParams)
{
  k_iter_++;

  updateJointsConfiguration(ctl);
  odometryRobot().posW(fbPose_);

  // we set the velocity and acceleration to zero as they will be compensated anyway as we compute the
  // successive poses in the local frame
  sva::MotionVecd zeroMotion = sva::MotionVecd::Zero();

  odometryRobot().velW(zeroMotion);
  odometryRobot().accW(zeroMotion);

  odometryRobot().forwardKinematics();
  odometryRobot().forwardVelocity();
  odometryRobot().forwardAcceleration();

  initContacts(ctl, logger, runParams);

  k_data_ = k_iter_;
}

template<typename OnNewContactObserver,
         typename OnMaintainedContactObserver,
         typename OnRemovedContactObserver,
         typename OnAddedContactObserver>
void LeggedOdometryManager::initContacts(const mc_control::MCController & ctl,
                                         mc_rtc::Logger & logger,
                                         const RunParameters<OnNewContactObserver,
                                                             OnMaintainedContactObserver,
                                                             OnRemovedContactObserver,
                                                             OnAddedContactObserver> & runParams)
{
  // If the position and orientation of the floating base can be updated using contacts (that were already set on the
  // previous iteration), they are updated, else we keep the previous estimation. Then we estimate the pose of new
  // contacts using the obtained pose of the floating base.

  const auto & robot = ctl.robot(robotName_);

  double sumForces_position = 0.0;
  posUpdatable_ = false;
  newContacts_.clear();
  maintainedContacts_.clear();

  auto onNewContact = [this, &ctl, &logger, &runParams](LoContactWithSensor & newContact)
  {
    addContactLogEntries(ctl, logger, newContact);

    newContacts_.push_back(&newContact);
    if constexpr(!std::is_same_v<OnNewContactObserver, std::nullptr_t>) { (*runParams.onNewContactFn)(newContact); }
  };

  auto onMaintainedContact =
      [this, &robot, &runParams, &sumForces_position, &ctl](LoContactWithSensor & maintainedContact)
  {
    maintainedContacts_.push_back(&maintainedContact);
    maintainedContact.lifeTimeIncrement(ctl.timeStep);

    // current estimate of the pose of the robot in the world
    const stateObservation::kine::Kinematics worldFbPose =
        conversions::kinematics::fromSva(odometryRobot().posW(), odometryRobot().velW());

    // we update the kinematics of the contact in the world obtained from the floating base and the sensor reading
    const stateObservation::kine::Kinematics & worldContactKine =
        getContactKinematics(maintainedContact, robot.forceSensor(maintainedContact.name()));

    sumForces_position += maintainedContact.forceNorm();

    maintainedContact.contactFbKine_ = worldContactKine.getInverse() * worldFbPose;

    maintainedContact.worldFbKineFromRef_ = maintainedContact.worldRefKine_ * maintainedContact.contactFbKine_;

    if constexpr(!std::is_same_v<OnMaintainedContactObserver, std::nullptr_t>)
    {
      (*runParams.onMaintainedContactFn)(maintainedContact);
    }

    posUpdatable_ = true;
  };

  auto onRemovedContact = [this, &logger, &runParams](LoContactWithSensor & removedContact)
  {
    removeContactLogEntries(logger, removedContact);
    if constexpr(!std::is_same_v<OnRemovedContactObserver, std::nullptr_t>)
    {
      (*runParams.onRemovedContactFn)(removedContact);
    }
  };

  // detects the contacts currently set with the environment
  contactsManager().updateContacts(ctl, robotName_, onNewContact, onMaintainedContact, onRemovedContact,
                                   *runParams.onAddedContactFn);

  for(auto * mContact : maintainedContacts_)
  {
    if(sumForces_position > 1e-6) { mContact->lambda(mContact->forceNorm() / sumForces_position); }
    else { mContact->lambda(0.0); }
  }
}

} // namespace mc_state_observation::odometry
