#include <mc_rtc/logging.h>
#include "mc_state_observation/measurements/measurements.h"
#include <mc_state_observation/conversions/kinematics.h>

#include <mc_state_observation/odometry/LeggedOdometryManager.h>

namespace so = stateObservation;

namespace mc_state_observation::odometry
{

///////////////////////////////////////////////////////////////////////
/// -------------------------Legged Odometry---------------------------
///////////////////////////////////////////////////////////////////////

void LeggedOdometryManager::init(const mc_control::MCController & ctl,
                                 const Configuration & odomConfig,
                                 const ContactsManagerConfiguration & contactsConf)

{
  robotName_ = odomConfig.robotName_;
  const auto & robot = ctl.robot(robotName_);

  if(robot.mb().joint(0).dof() != 6)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("This robot does not have a floating base");
  }

  odometryRobot_ = mc_rbdyn::Robots::make();
  odometryRobot_->robotCopy(robot, "odometryRobot");

  odometryType_ = odomConfig.odometryType_;
  withYawEstimation_ = odomConfig.withYaw_;
  velocityUpdate_ = odomConfig.velocityUpdate_;
  odometryName_ = odomConfig.odometryName_;

  fbPose_.translation() = robot.posW().translation();
  fbPose_.rotation() = robot.posW().rotation();

  contactsManager_.init(ctl, robotName_, contactsConf);

  if(!ctl.datastore().has("KinematicAnchorFrame::" + ctl.robot(robotName_).name()))
  {
    if(!robot.hasSurface("LeftFootCenter") || !robot.hasSurface("RightFootCenter"))
    {
      mc_rtc::log::error_and_throw("The surfaces used to compute the anchor frame don't exist in this robot.");
    }

    double leftFootRatio = robot.indirectSurfaceForceSensor("LeftFootCenter").force().z()
                           / (robot.indirectSurfaceForceSensor("LeftFootCenter").force().z()
                              + robot.indirectSurfaceForceSensor("RightFootCenter").force().z());

    worldAnchorPose_ = conversions::kinematics::fromSva(
        sva::interpolate(robot.surfacePose("RightFootCenter"), robot.surfacePose("LeftFootCenter"), leftFootRatio),
        so::kine::Kinematics::Flags::pose);
  }
  else
  {
    worldAnchorPose_ = conversions::kinematics::fromSva(
        ctl.datastore().call<sva::PTransformd>("KinematicAnchorFrame::" + ctl.robot(robotName_).name(),
                                               ctl.robot(robotName_)),
        so::kine::Kinematics::Flags::pose);
  }

  auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();
  logger.addLogEntry(odometryName_ + "_odometryRobot_posW",
                     [this]() -> const sva::PTransformd & { return odometryRobot().posW(); });

  logger.addLogEntry(odometryName_ + "_odometryRobot_velW",
                     [this]() -> const sva::MotionVecd & { return odometryRobot().velW(); });

  logger.addLogEntry(odometryName_ + "_odometryRobot_accW", [this]() { return odometryRobot().accW(); });
  if(odomConfig.withModeSwitchInGui_)
  {
    ctl.gui()->addElement({odometryName_, "Odometry"},
                          mc_rtc::gui::ComboInput(
                              "Choose from list",
                              {measurements::odometryTypeToSstring(measurements::OdometryType::Odometry6d),
                               measurements::odometryTypeToSstring(measurements::OdometryType::Flat)},
                              [this]() -> std::string { return measurements::odometryTypeToSstring(odometryType_); },
                              [this](const std::string & typeOfOdometry)
                              { setOdometryType(measurements::stringToOdometryType(typeOfOdometry)); }));

    logger.addLogEntry(odometryName_ + "_debug_OdometryType",
                       [this]() -> std::string { return measurements::odometryTypeToSstring(odometryType_); });
  }
}

void LeggedOdometryManager::updateJointsConfiguration(const mc_control::MCController & ctl)
{
  const auto & realRobot = ctl.realRobot(robotName_);

  // Copy the real configuration except for the floating base
  const auto & realQ = realRobot.mbc().q;
  const auto & realAlpha = realRobot.mbc().alpha;

  std::copy(std::next(realQ.begin()), realQ.end(), std::next(odometryRobot().mbc().q.begin()));
  std::copy(std::next(realAlpha.begin()), realAlpha.end(), std::next(odometryRobot().mbc().alpha.begin()));

  odometryRobot().forwardKinematics();
  odometryRobot().forwardVelocity();
}

void LeggedOdometryManager::run(const mc_control::MCController & ctl, mc_rtc::Logger & logger, sva::PTransformd & pose)
{
  // the tilt must come from another estimator so we will use the real robot for the orientation
  runPvt(ctl, logger, pose);
}

void LeggedOdometryManager::run(const mc_control::MCController & ctl,
                                mc_rtc::Logger & logger,
                                sva::PTransformd & pose,
                                sva::MotionVecd & vel)
{
  // the tilt must come from another estimator so we will use the real robot for the orientation
  runPvt(ctl, logger, pose, nullptr, &vel);
}

void LeggedOdometryManager::run(const mc_control::MCController & ctl,
                                mc_rtc::Logger & logger,
                                sva::PTransformd & pose,
                                sva::MotionVecd & vel,
                                sva::MotionVecd & acc)
{
  // the tilt must come from another estimator so we will use the real robot for the orientation
  runPvt(ctl, logger, pose, nullptr, &vel, &acc);
}

void LeggedOdometryManager::run(const mc_control::MCController & ctl,
                                mc_rtc::Logger & logger,
                                sva::PTransformd & pose,
                                const stateObservation::Matrix3 & tilt)
{
  runPvt(ctl, logger, pose, &tilt);
}

void LeggedOdometryManager::run(const mc_control::MCController & ctl,
                                mc_rtc::Logger & logger,
                                sva::PTransformd & pose,
                                const stateObservation::Matrix3 & tilt,
                                sva::MotionVecd & vel)
{
  runPvt(ctl, logger, pose, &tilt, &vel);
}

void LeggedOdometryManager::run(const mc_control::MCController & ctl,
                                mc_rtc::Logger & logger,
                                sva::PTransformd & pose,
                                const stateObservation::Matrix3 & tilt,
                                sva::MotionVecd & vel,
                                sva::MotionVecd & acc)
{
  runPvt(ctl, logger, pose, &tilt, &vel, &acc);
}

void LeggedOdometryManager::runPvt(const mc_control::MCController & ctl,
                                   mc_rtc::Logger & logger,
                                   sva::PTransformd & pose,
                                   const stateObservation::Matrix3 * tilt,
                                   sva::MotionVecd * vel,
                                   sva::MotionVecd * acc)
{
  updateJointsConfiguration(ctl);
  odometryRobot().posW(fbPose_);

  // we set the velocity and acceleration to zero as they will be compensated anyway as we compute the
  // successive poses in the local frame
  sva::MotionVecd zeroMotion;
  zeroMotion.linear() = so::Vector3::Zero();
  zeroMotion.angular() = so::Vector3::Zero();
  odometryRobot().velW(zeroMotion);
  odometryRobot().accW(zeroMotion);

  odometryRobot().forwardKinematics();
  odometryRobot().forwardVelocity();
  odometryRobot().forwardAcceleration();

  // updates the contacts and the resulting floating base kinematics
  if(tilt != nullptr) { updateFbAndContacts(ctl, logger, *tilt, vel, acc); }
  else
  {
    // the tilt must come from another estimator so we will use the real robot for the orientation
    const auto & realRobot = ctl.realRobot(robotName_);
    so::Matrix3 realRobotOri = realRobot.posW().rotation().transpose();

    updateFbAndContacts(ctl, logger, realRobotOri, vel, acc);
  }

  // updates the floating base kinematics in the observer
  updateFbKinematicsPvt(pose, vel, acc);
}

void LeggedOdometryManager::updateFbKinematicsPvt(sva::PTransformd & pose, sva::MotionVecd * vel, sva::MotionVecd * acc)
{
  pose.rotation() = odometryRobot().posW().rotation();
  pose.translation() = odometryRobot().posW().translation();

  // we express the velocity and acceleration computed by the previous obervers in our newly estimated frame.
  // even if the velocity is estimated, it will be updated only if
  if(vel != nullptr)
  {
    vel->linear() = odometryRobot().velW().linear();
    vel->angular() = odometryRobot().velW().angular();
  }
  if(acc != nullptr)
  {
    acc->linear() = odometryRobot().accW().linear();
    acc->angular() = odometryRobot().accW().angular();
  }
}

void LeggedOdometryManager::updateFbAndContacts(const mc_control::MCController & ctl,
                                                mc_rtc::Logger & logger,
                                                const stateObservation::Matrix3 & tilt,
                                                sva::MotionVecd * vel,
                                                sva::MotionVecd * acc)
{
  // If the position and orientation of the floating base can be updated using contacts (that were already set on the
  // previous iteration), they are updated, else we keep the previous estimation. Then we estimate the pose of new
  // contacts using the obtained pose of the floating base.

  const auto & robot = ctl.robot(robotName_);

  double sumForces_position = 0.0;
  double sumForces_orientation = 0.0;

  // indicates if the position can be updated from the current contacts or not
  bool posUpdatable = false;
  // indicates if the orientation can be updated from the current contacts or not
  bool oriUpdatable = false;

  // force weighted sum of the estimated floating base positions
  so::Vector3 totalFbPosition = so::Vector3::Zero();

  // current estimate of the pose of the robot in the world
  const so::kine::Kinematics worldFbPose =
      conversions::kinematics::fromSva(odometryRobot().posW(), so::kine::Kinematics::Flags::pose);

  auto onNewContact = [this, &logger](LoContactWithSensor & newContact) { addContactLogEntries(logger, newContact); };
  auto onMaintainedContact = [this, &robot, &posUpdatable, &worldFbPose, &sumForces_position,
                              &totalFbPosition](LoContactWithSensor & maintainedContact)
  {
    /* For each maintained contact, we compute the position of the floating base in the world, we then compute the
     * weighted average wrt to the measured forces at the contact and obtain the estimated position of the floating
     * base */

    // indicates that we can compute the position of the floating base using the contacts
    posUpdatable = true;

    sumForces_position += maintainedContact.forceNorm();

    // kinematics of the contact obtained from the floating base
    const so::kine::Kinematics & worldContactKine =
        getCurrentContactKinematics(maintainedContact, robot.forceSensor(maintainedContact.name()));

    // we compute the current position of the floating base in the world from the reference position of the contact and
    // the transformation from the contact to the floating base obtained from the encoders
    maintainedContact.currentWorldFbPose_.position =
        maintainedContact.worldRefKine_.position() + (worldFbPose.position() - worldContactKine.position());

    // we add the position of the floating base estimated from the one of the contact to the sum
    totalFbPosition += maintainedContact.currentWorldFbPose_.position() * maintainedContact.forceNorm();
  };

  auto onRemovedContact = [this, &logger](LoContactWithSensor & removedContact)
  { removeContactLogEntries(logger, removedContact); };

  // detects the contacts currently set with the environment
  contactsManager().updateContacts(ctl, robotName_, onNewContact, onMaintainedContact, onRemovedContact);

  // if we can update the position, we compute the weighted average of the position obtained from the contacts
  if(posUpdatable) { fbPose_.translation() = totalFbPosition / sumForces_position; }

  // selects the contacts to use for the yaw odometry. We cannot call it in the onMaintainedContact function as it is
  // looping over all the maintained contact and not used on each contact separately
  selectForOrientationOdometry(oriUpdatable, sumForces_orientation, worldFbPose);

  if(oriUpdatable)
  {
    // the orientation can be updated using contacts, it will use at most the two most suitable contacts.
    // We merge the obtained yaw with the tilt estimated by the previous observers
    if(contactsManager_.oriOdometryContacts_.size() == 1)
    {
      // the orientation can be updated using 1 contact
      fbPose_.rotation() =
          so::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(
              tilt, contactsManager_.oriOdometryContacts_.begin()->get().currentWorldFbPose_.orientation)
              .transpose();
    }
    if(contactsManager_.oriOdometryContacts_.size() == 2) // the orientation can be updated using 2 contacts
    {
      const auto & contact1 = (*contactsManager_.oriOdometryContacts_.begin()).get();
      const auto & contact2 = (*std::next(contactsManager_.oriOdometryContacts_.begin(), 1)).get();

      const auto & R1 = contact1.currentWorldFbPose_.orientation.toMatrix3();
      const auto & R2 = contact2.currentWorldFbPose_.orientation.toMatrix3();

      double u = contact1.forceNorm() / sumForces_orientation;

      so::Matrix3 diffRot = R1.transpose() * R2;

      so::Vector3 diffRotVector = (1.0 - u)
                                  * so::kine::skewSymmetricToRotationVector(
                                      diffRot); // we perform the multiplication by the weighting coefficient now so a
                                                // zero coefficient gives a unit rotation matrix and not a zero matrix

      so::AngleAxis diffRotAngleAxis = so::kine::rotationVectorToAngleAxis(diffRotVector);

      so::Matrix3 diffRotMatrix = so::kine::Orientation(diffRotAngleAxis).toMatrix3(); // exp( (1 - u) * log(R1^T R2) )

      so::Matrix3 meanOri = R1 * diffRotMatrix;

      fbPose_.rotation() = so::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(tilt, meanOri).transpose();
    }
  }
  else
  {
    // If no contact is detected, the yaw will not be updated but the tilt will.
    fbPose_.rotation() =
        so::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(tilt, fbPose_.rotation().transpose()).transpose();
  }

  // update of the pose of the floating base of the odometry robot in the world frame before creating the new contacts
  updateOdometryRobot(ctl, vel, acc);

  // computation of the reference kinematics of the newly set contacts in the world. We cannot use the onNewContacts
  // function as it is used at the beginning of the iteration and we need to compute this at the end
  for(const int & foundContactIndex : contactsManager().contactsFound())
  {
    if(!contactsManager().contact(foundContactIndex).wasAlreadySet()) // the contact was not set so we will
                                                                      // compute its kinematics
    {
      LoContactWithSensor & foundContact = contactsManager_.contact(foundContactIndex);

      setNewContact(foundContact, robot);
    }
  }
}

void LeggedOdometryManager::updateOdometryRobot(const mc_control::MCController & ctl,
                                                sva::MotionVecd * vel,
                                                sva::MotionVecd * acc)
{
  const auto & realRobot = ctl.realRobot(robotName_);

  // new estimated orientation of the floating base.
  so::kine::Orientation newOri(so::Matrix3(fbPose_.rotation().transpose()));

  // if an acceleration was already estimated, we express it in the new estimated robot
  if(acc != nullptr)
  {
    // realRobot.posW().rotation() is the transpose of R
    so::Vector3 realLocalLinAcc = realRobot.posW().rotation() * realRobot.accW().linear();
    so::Vector3 realLocalAngAcc = realRobot.posW().rotation() * realRobot.accW().angular();
    sva::MotionVecd acc;

    acc.linear() = newOri * realLocalLinAcc;
    acc.angular() = newOri * realLocalAngAcc;

    odometryRobot().accW(acc);
  }

  // if a velocity was already estimated, we express it in the new estimated robot. Otherwise we estimate it with
  // finite differences
  if(vel != nullptr)
  {
    if(velocityUpdate_ == VelocityUpdate::FromUpstream)
    {
      // realRobot.posW().rotation() is the transpose of R
      so::Vector3 realLocalLinVel = realRobot.posW().rotation() * realRobot.velW().linear();
      so::Vector3 realLocalAngVel = realRobot.posW().rotation() * realRobot.velW().angular();

      sva::MotionVecd vel;

      vel.linear() = newOri * realLocalLinVel;
      vel.angular() = newOri * realLocalAngVel;
      odometryRobot().velW(vel);
    }
    if(velocityUpdate_ == VelocityUpdate::FiniteDiff)
    {
      sva::MotionVecd vel;

      vel.linear() = (fbPose_.translation() - odometryRobot().posW().translation()) / ctl.timeStep;
      so::kine::Orientation oldOri(so::Matrix3(odometryRobot().posW().rotation().transpose()));
      vel.angular() = oldOri.differentiate(newOri) / ctl.timeStep;
      odometryRobot().velW(vel);
    }
  }

  // modified at the end as we might need the previous pose to get the velocity by finite differences.
  odometryRobot().posW(fbPose_);

  odometryRobot().forwardKinematics();

  odometryRobot().forwardVelocity();
  if(acc != nullptr) { odometryRobot().forwardAcceleration(); }
}

void LeggedOdometryManager::setNewContact(LoContactWithSensor & contact, const mc_rbdyn::Robot & measurementsRobot)
{
  const mc_rbdyn::ForceSensor & forceSensor = measurementsRobot.forceSensor(contact.forceSensor());
  // If the contact is not detected using surfaces, we must consider that the frame of the sensor is the one of the
  // surface).

  if(contactsManager_.getContactsDetection() == ContactsManager::ContactsDetection::Sensors)
  {
    so::kine::Kinematics worldNewContactKineOdometryRobot;
    so::kine::Kinematics worldContactKineRef;
    worldContactKineRef.setZero(so::kine::Kinematics::Flags::position);

    // getting the position in the world of the new contact
    const sva::PTransformd & bodyNewContactPoseRobot = forceSensor.X_p_f();
    so::kine::Kinematics bodyNewContactKine;
    bodyNewContactKine.setZero(so::kine::Kinematics::Flags::pose);
    bodyNewContactKine.position = bodyNewContactPoseRobot.translation();
    bodyNewContactKine.orientation = so::Matrix3(bodyNewContactPoseRobot.rotation().transpose());

    so::kine::Kinematics worldBodyKineOdometryRobot;

    worldBodyKineOdometryRobot.position =
        odometryRobot().mbc().bodyPosW[odometryRobot().bodyIndexByName(forceSensor.parentBody())].translation();
    worldBodyKineOdometryRobot.orientation =
        so::Matrix3(odometryRobot()
                        .mbc()
                        .bodyPosW[odometryRobot().bodyIndexByName(forceSensor.parentBody())]
                        .rotation()
                        .transpose());

    worldNewContactKineOdometryRobot = worldBodyKineOdometryRobot * bodyNewContactKine;

    contact.worldRefKine_.position = worldNewContactKineOdometryRobot.position();
    contact.worldRefKine_.orientation = worldNewContactKineOdometryRobot.orientation;
  }
  else // the kinematics of the contact are directly the ones of the surface
  {
    sva::PTransformd worldSurfacePoseOdometryRobot = odometryRobot().surfacePose(contact.surface());

    contact.worldRefKine_.position = worldSurfacePoseOdometryRobot.translation();
    contact.worldRefKine_.orientation = so::Matrix3(worldSurfacePoseOdometryRobot.rotation().transpose());
  }

  if(odometryType_ == measurements::OdometryType::Flat) { contact.worldRefKine_.position()(2) = 0.0; }
}

const so::kine::Kinematics & LeggedOdometryManager::getCurrentContactKinematics(LoContactWithSensor & contact,
                                                                                const mc_rbdyn::ForceSensor & fs)
{
  // robot is necessary because odometry robot doesn't have the copy of the force measurements
  const sva::PTransformd & bodyContactSensorPose = fs.X_p_f();
  so::kine::Kinematics bodyContactSensorKine =
      conversions::kinematics::fromSva(bodyContactSensorPose, so::kine::Kinematics::Flags::vel);

  // kinematics of the sensor's parent body in the world
  so::kine::Kinematics worldBodyKineOdometryRobot =
      conversions::kinematics::fromSva(odometryRobot().mbc().bodyPosW[odometryRobot().bodyIndexByName(fs.parentBody())],
                                       so::kine::Kinematics::Flags::pose);

  so::kine::Kinematics worldSensorKineOdometryRobot = worldBodyKineOdometryRobot * bodyContactSensorKine;

  if(contactsManager_.getContactsDetection() == ContactsManager::ContactsDetection::Sensors)
  {
    // If the contact is detecting using thresholds, we will then consider the sensor frame as
    // the contact surface frame directly.
    contact.currentWorldKine_ = worldSensorKineOdometryRobot;
  }
  else // the kinematics of the contact are the ones of the associated surface
  {
    // the kinematics of the contacts are the ones of the surface, but we must transport the measured wrench
    sva::PTransformd worldSurfacePoseOdometryRobot = odometryRobot().surfacePose(contact.surface());
    contact.currentWorldKine_ =
        conversions::kinematics::fromSva(worldSurfacePoseOdometryRobot, so::kine::Kinematics::Flags::pose);

    so::kine::Kinematics contactSensorKine = contact.currentWorldKine_.getInverse() * worldSensorKineOdometryRobot;
    // expressing the force measurement in the frame of the surface
    contact.forceNorm((contactSensorKine.orientation * fs.wrenchWithoutGravity(odometryRobot()).force()).norm());
  }

  return contact.currentWorldKine_;
}

void LeggedOdometryManager::selectForOrientationOdometry(bool & oriUpdatable,
                                                         double & sumForcesOrientation,
                                                         const stateObservation::kine::Kinematics & worldFbPose)
{
  // if the estimation of yaw is not required, we don't need to select the contacts
  if(withYawEstimation_)
  {
    contactsManager_.oriOdometryContacts_.clear();
    for(auto it = contactsManager_.contactsFound().begin(); it != contactsManager_.contactsFound().end(); it++)
    {
      LoContactWithSensor & contact = contactsManager_.contact(*it);
      if(contact.name().find("Hand") == std::string::npos
         && contact.wasAlreadySet()) // we don't use hands for the orientation odometry
      {
        contact.useForOrientation_ = true;
        contactsManager_.oriOdometryContacts_.insert(contact);
      }
    }
    // contacts are sorted from the lowest force to the highest force
    while(contactsManager_.oriOdometryContacts_.size() > 2)
    {
      (*contactsManager_.oriOdometryContacts_.begin()).get().useForOrientation_ = false;
      contactsManager_.oriOdometryContacts_.erase(contactsManager_.oriOdometryContacts_.begin());
    }

    // the position of the floating base in the world can be obtained by a weighted average of the estimations for each
    // contact
    for(LoContactWithSensor & oriOdomContact : contactsManager_.oriOdometryContacts_)
    {
      // the orientation can be computed using contacts
      oriUpdatable = true;

      sumForcesOrientation += oriOdomContact.forceNorm();

      so::Matrix3 contactFrameOri_odometryRobot =
          oriOdomContact.currentWorldKine_.orientation.toMatrix3().transpose() * worldFbPose.orientation.toMatrix3();
      oriOdomContact.currentWorldFbPose_.orientation =
          so::Matrix3(oriOdomContact.worldRefKine_.orientation.toMatrix3() * contactFrameOri_odometryRobot);
    }
  }
}

void LeggedOdometryManager::addContactLogEntries(mc_rtc::Logger & logger, const LoContactWithSensor & contact)
{
  const std::string & contactName = contact.name();
  conversions::kinematics::addToLogger(logger, contact.worldRefKine_, odometryName_ + "_" + contactName + "_refPose");
  conversions::kinematics::addToLogger(logger, contact.currentWorldFbPose_,
                                       odometryName_ + "_" + contactName + "_currentWorldFbPose");
  conversions::kinematics::addToLogger(logger, contact.currentWorldKine_,
                                       odometryName_ + "_" + contactName + "_currentWorldContactKine");
}

void LeggedOdometryManager::removeContactLogEntries(mc_rtc::Logger & logger, const LoContactWithSensor & contact)
{
  const std::string & contactName = contact.name();
  logger.removeLogEntry(odometryName_ + "_" + contactName + "_ref_position");
  logger.removeLogEntry(odometryName_ + "_" + contactName + "_ref_orientation");
  conversions::kinematics::removeFromLogger(logger, contact.worldRefKine_);
  conversions::kinematics::removeFromLogger(logger, contact.currentWorldFbPose_);
  conversions::kinematics::removeFromLogger(logger, contact.currentWorldKine_);
}

so::kine::Kinematics & LeggedOdometryManager::getAnchorFramePose(const mc_control::MCController & ctl,
                                                                 const std::string & bodySensorName)
{
  const auto & robot = ctl.robot(robotName_);

  double sumForces_position = 0.0;
  double sumForces_orientation = 0.0;

  bool posUpdatable = false;
  bool oriUpdatable = false;

  // "force-weighted" sum of the estimated floating base positions
  so::Vector3 totalAnchorPosition = so::Vector3::Zero();

  worldAnchorPose_.reset();

  // checks that the position and orientation can be updated from the currently set contacts and computes the pose of
  // the floating base obtained from each contact
  for(const int & setContactIndex : contactsManager().contactsFound())
  {
    if(contactsManager()
           .contact(setContactIndex)
           .wasAlreadySet()) // the contact already exists so we will use it to estimate the floating base pose
    {
      posUpdatable = true;

      LoContactWithSensor & setContact = contactsManager_.contact(setContactIndex);
      const so::kine::Kinematics & worldContactKine =
          getCurrentContactKinematics(setContact, robot.forceSensor(setContact.name()));

      sumForces_position += setContact.forceNorm();
      // force weighted sum of the estimated floating base positions
      totalAnchorPosition += worldContactKine.position() * setContact.forceNorm();

      if(withYawEstimation_ && setContact.useForOrientation_)
      {
        oriUpdatable = true;

        sumForces_orientation += setContact.forceNorm();
      }
    }
  }

  if(posUpdatable)
  {
    worldAnchorPose_.position = totalAnchorPosition / sumForces_position;

    if(!prevAnchorFromContacts_)
    {
      worldAnchorPose_.linVel.set().setZero();
      worldAnchorPose_.angVel.set().setZero();
      prevAnchorFromContacts_ = true;
    }
  }
  else
  {
    // if we cannot update the position (so not the orientations either) using contacts, we use the IMU frame as the
    // anchor frame.
    const auto & imu = ctl.robot(robotName_).bodySensor(bodySensorName);

    const sva::PTransformd & imuXbs = imu.X_b_s();
    so::kine::Kinematics parentImuKine = conversions::kinematics::fromSva(imuXbs, so::kine::Kinematics::Flags::pose);

    const sva::PTransformd & parentPoseW = odometryRobot().bodyPosW(imu.parentBody());

    so::kine::Kinematics worldParentKine =
        conversions::kinematics::fromSva(parentPoseW, so::kine::Kinematics::Flags::pose);

    // pose of the IMU in the world frame
    worldAnchorPose_ = worldParentKine * parentImuKine;

    if(prevAnchorFromContacts_)
    {
      worldAnchorPose_.linVel.set().setZero();
      worldAnchorPose_.angVel.set().setZero();
      prevAnchorFromContacts_ = false;
    }
  }

  if(oriUpdatable)
  {
    if(contactsManager_.oriOdometryContacts_.size() == 1) // the orientation can be updated using 1 contact
    {
      worldAnchorPose_.orientation = contactsManager_.oriOdometryContacts_.begin()->get().currentWorldKine_.orientation;
    }
    if(contactsManager_.oriOdometryContacts_.size() == 2) // the orientation can be updated using 2 contacts
    {
      const auto & contact1 = (*contactsManager_.oriOdometryContacts_.begin()).get();
      const auto & contact2 = (*std::next(contactsManager_.oriOdometryContacts_.begin(), 1)).get();

      const auto & R1 = contact1.currentWorldKine_.orientation.toMatrix3();
      const auto & R2 = contact2.currentWorldKine_.orientation.toMatrix3();

      double u;

      u = contact1.forceNorm() / sumForces_orientation;

      so::Matrix3 diffRot = R1.transpose() * R2;

      so::Vector3 diffRotVector = (1.0 - u)
                                  * so::kine::skewSymmetricToRotationVector(
                                      diffRot); // we perform the multiplication by the weighting coefficient now so a
                                                // zero coefficient gives a unit rotation matrix and not a zero matrix
      so::AngleAxis diffRotAngleAxis = so::kine::rotationVectorToAngleAxis(diffRotVector);

      so::Matrix3 diffRotMatrix = so::kine::Orientation(diffRotAngleAxis).toMatrix3(); // exp( (1 - u) * log(R1^T R2) )

      so::Matrix3 meanOri = R1 * diffRotMatrix;

      worldAnchorPose_.orientation = meanOri;
    }
  }
  else
  {
    const auto & imu = ctl.robot(robotName_).bodySensor(bodySensorName);
    const sva::PTransformd & imuXbs = imu.X_b_s();
    so::kine::Kinematics parentImuKine = conversions::kinematics::fromSva(imuXbs, so::kine::Kinematics::Flags::pose);

    const sva::PTransformd & parentPoseW = odometryRobot().bodyPosW(imu.parentBody());

    so::kine::Kinematics worldParentKine =
        conversions::kinematics::fromSva(parentPoseW, so::kine::Kinematics::Flags::pose);

    // pose of the IMU in the world frame
    so::kine::Kinematics worldImuKine = worldParentKine * parentImuKine;
    worldAnchorPose_.orientation = worldImuKine.orientation;
  }

  return worldAnchorPose_;
}

void LeggedOdometryManager::setOdometryType(OdometryType newOdometryType)
{

  OdometryType prevOdometryType = odometryType_;
  odometryType_ = newOdometryType;

  if(odometryType_ != prevOdometryType)
  {
    mc_rtc::log::info("[{}]: Odometry mode changed to: {}", odometryName_,
                      measurements::odometryTypeToSstring(newOdometryType));
  }
}

} // namespace mc_state_observation::odometry
