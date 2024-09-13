#include <mc_rtc/logging.h>

#include <mc_state_observation/measurements/measurements.h>

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
  correctContacts_ = odomConfig.correctContacts_;
  velocityUpdate_ = odomConfig.velocityUpdate_;
  odometryName_ = odomConfig.odometryName_;

  fbPose_.translation() = robot.posW().translation();
  fbPose_.rotation() = robot.posW().rotation();

  contactsManager_.init(ctl, robotName_, contactsConf);

  sva::PTransformd worldAnchorKine;
  if(!ctl.datastore().has("KinematicAnchorFrame::" + ctl.robot(robotName_).name()))
  {
    if(!robot.hasSurface("LeftFootCenter") || !robot.hasSurface("RightFootCenter"))
    {
      mc_rtc::log::error_and_throw("The surfaces used to compute the anchor frame don't exist in this robot.");
    }

    double leftFootRatio = robot.indirectSurfaceForceSensor("LeftFootCenter").force().z()
                           / (robot.indirectSurfaceForceSensor("LeftFootCenter").force().z()
                              + robot.indirectSurfaceForceSensor("RightFootCenter").force().z());

    worldAnchorKine =
        sva::interpolate(robot.surfacePose("RightFootCenter"), robot.surfacePose("LeftFootCenter"), leftFootRatio);
  }
  else
  {
    worldAnchorKine = ctl.datastore().call<sva::PTransformd>("KinematicAnchorFrame::" + ctl.robot(robotName_).name(),
                                                             ctl.robot(robotName_));
  }
  worldRefAnchorPosition_ = worldAnchorKine.translation();
  worldAnchorPos_ = worldAnchorKine.translation();

  fbAnchorPos_ =
      -robot.posW().rotation() * robot.posW().translation() + robot.posW().rotation().transpose() * worldAnchorPos_;

  if(odomConfig.withModeSwitchInGui_)
  {
    std::vector<std::string> odomCategory;
    odomCategory.insert(odomCategory.end(),
                        {"ObserverPipelines", ctl.observerPipeline().name(), odometryName_, "Odometry"});

    ctl.gui()->addElement({odomCategory},
                          mc_rtc::gui::ComboInput(
                              "Choose from list",
                              {measurements::odometryTypeToSstring(measurements::OdometryType::Odometry6d),
                               measurements::odometryTypeToSstring(measurements::OdometryType::Flat)},
                              [this]() -> std::string { return measurements::odometryTypeToSstring(odometryType_); },
                              [this](const std::string & typeOfOdometry)
                              { setOdometryType(measurements::stringToOdometryType(typeOfOdometry)); }));
  }
}

void LeggedOdometryManager::reset()
{
  worldRefAnchorPosition_ = sva::interpolate(odometryRobot().surfacePose("RightFootCenter"),
                                             odometryRobot().surfacePose("LeftFootCenter"), 0.5)
                                .translation();

  fbAnchorPos_ = -odometryRobot().posW().rotation() * odometryRobot().posW().translation()
                 + odometryRobot().posW().rotation().transpose() * worldRefAnchorPosition_;
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

void LeggedOdometryManager::run(const mc_control::MCController & ctl, KineParams & kineParams)
{
  if(k_data_ == k_est_) { mc_rtc::log::error_and_throw("Please call initLoop before this function"); }

  if(kineParams.tiltOrAttitude == nullptr)
  { // the tilt must come from another estimator so we will use the real robot for the orientation
    const auto & realRobot = ctl.realRobot(robotName_);
    const stateObservation::Matrix3 realRobotOri = realRobot.posW().rotation().transpose();
    kineParams.tilt(realRobotOri);
  }

  // updates the contacts and the resulting floating base kinematics
  updateFbAndContacts(ctl, kineParams);

  // updates the floating base kinematics in the observer
  updateFbKinematicsPvt(kineParams.pose, kineParams.vel, kineParams.acc);

  k_est_ = k_iter_;
}

void LeggedOdometryManager::updateFbAndContacts(const mc_control::MCController & ctl, const KineParams & params)
{
  // If the position and orientation of the floating base can be updated using contacts (that were already set on the
  // previous iteration), they are updated, else we keep the previous estimation. Then we estimate the pose of new
  // contacts using the obtained pose of the floating base.

  const auto & robot = ctl.robot(robotName_);

  double sumForces_orientation = 0.0;

  // indicates if the orientation can be updated from the current contacts or not
  bool oriUpdatable = false;

  // if the given orientation is only a tilt, we compute the yaw using the one of the contacts
  if(!params.oriIsAttitude)
  {
    const stateObservation::Matrix3 & tilt = *(params.tiltOrAttitude);
    // selects the contacts to use for the yaw odometry. We cannot call it in the onMaintainedContact function as it is
    // looping over all the maintained contact and not used on each contact separately
    selectForOrientationOdometry(oriUpdatable, sumForces_orientation);

    // we update the orientation of the floating base first
    if(oriUpdatable)
    {
      // the orientation can be updated using contacts, it will use at most the two most suitable contacts.
      // We merge the obtained yaw with the tilt estimated by the previous observers
      if(contactsManager_.oriOdometryContacts_.size() == 1)
      {
        // the orientation can be updated using 1 contact
        fbPose_.rotation() =
            stateObservation::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(
                tilt, contactsManager_.oriOdometryContacts_.begin()->get().worldFbKineFromRef_.orientation)
                .transpose();
      }
      if(contactsManager_.oriOdometryContacts_.size() == 2) // the orientation can be updated using 2 contacts
      {
        const auto & contact1 = (*contactsManager_.oriOdometryContacts_.begin()).get();
        const auto & contact2 = (*std::next(contactsManager_.oriOdometryContacts_.begin(), 1)).get();

        const auto & R1 = contact1.worldFbKineFromRef_.orientation.toMatrix3();
        const auto & R2 = contact2.worldFbKineFromRef_.orientation.toMatrix3();

        double u = contact1.forceNorm() / sumForces_orientation;

        stateObservation::Matrix3 diffRot = R1.transpose() * R2;

        stateObservation::Vector3 diffRotVector =
            (1.0 - u)
            * stateObservation::kine::skewSymmetricToRotationVector(
                diffRot); // we perform the multiplication by the weighting coefficient now so a
                          // zero coefficient gives a unit rotation matrix and not a zero matrix

        stateObservation::AngleAxis diffRotAngleAxis = stateObservation::kine::rotationVectorToAngleAxis(diffRotVector);

        stateObservation::Matrix3 diffRotMatrix =
            stateObservation::kine::Orientation(diffRotAngleAxis).toMatrix3(); // exp( (1 - u) * log(R1^T R2) )

        stateObservation::Matrix3 meanOri = R1 * diffRotMatrix;
        fbPose_.rotation() = stateObservation::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(tilt, meanOri).transpose();
      }
    }
    else
    {
      // If no contact is detected, the yaw will not be updated but the tilt will.
      fbPose_.rotation() =
          stateObservation::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(tilt, fbPose_.rotation().transpose()).transpose();
    }
  }
  else
  {
    const stateObservation::Matrix3 & attitude = *(params.tiltOrAttitude);
    fbPose_.rotation() = attitude.transpose();
  }

  // we update the orientation of the floating base
  odometryRobot().posW(fbPose_);

  /*   Update of the position of the floating base    */
  if(params.worldPos != nullptr)
  {
    /* If exceptionally the position in the world is given, we use it directly */
    fbPose_.translation() = *(params.worldPos);
  }
  else
  {
    // we need to update the robot's configuration after the update of the orientation
    odometryRobot().forwardKinematics();
    /*   if we can update the position, we compute the weighted average of the position obtained from the contacts    */
    updatePositionOdometry();
  }
  updateOdometryRobot(ctl, params.vel, params.acc);

  // we correct the reference position of the contacts in the world
  if(correctContacts_) { correctContactsRef(); }

  // computation of the reference kinematics of the newly set contacts in the world. We cannot use the onNewContacts
  // function as it is used at the beginning of the iteration and we need to compute this at the end
  for(auto * nContact : newContacts_) { setNewContact(*nContact, robot); }
}

void LeggedOdometryManager::selectForOrientationOdometry(bool & oriUpdatable, double & sumForcesOrientation)
{
  // we cannot update the orientation if no contact was set on last iteration
  if(!posUpdatable_) { return; }

  // if the estimation of yaw is not required, we don't need to select the contacts
  if(withYawEstimation_)
  {
    contactsManager_.oriOdometryContacts_.clear();
    for(auto * mContact : maintainedContacts_)
    {
      if(mContact->name().find("Hand") == std::string::npos && mContact->isSet()
         && mContact->wasAlreadySet()) // we don't use hands for the orientation odometry
      {
        mContact->useForOrientation_ = true;
        contactsManager_.oriOdometryContacts_.insert(*mContact);
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

      oriOdomContact.worldFbKineFromRef_.orientation = so::Matrix3(
          oriOdomContact.worldRefKine_.orientation.toMatrix3() * oriOdomContact.contactFbKine_.orientation.toMatrix3());
    }
  }
}

void LeggedOdometryManager::updatePositionOdometry()
{
  /* For each maintained contact, we compute the position of the floating base in the contact frame, we then compute the
   * weighted average wrt to the measured forces at the contact and obtain the estimated translation from the anchor
   * point to the floating base.  We apply this translation to the reference position of the anchor frame in the world
   * to obtain the new position of the floating base in the word. */

  if(posUpdatable_) { fbPose_.translation() = getWorldFbPosFromAnchor(); }
}

so::Vector3 LeggedOdometryManager::getWorldFbPosFromAnchor()
{
  /* For each maintained contact, we compute the position of the floating base in the contact frame, we then compute the
   * weighted average wrt to the measured forces at the contact and obtain the estimated translation from the anchor
   * point to the floating base.  We apply this translation to the reference position of the anchor frame in the world
   * to obtain the new position of the floating base in the word. */

  fbAnchorPos_.setZero();
  so::Vector3 worldFbPosFromAnchor;

  for(auto * mContact : maintainedContacts_)
  {
    fbAnchorPos_ += mContact->contactFbKine_.getInverse().position() * mContact->lambda();
  }

  worldFbPosFromAnchor = getWorldRefAnchorPos() - fbPose_.rotation().transpose() * fbAnchorPos_;

  return worldFbPosFromAnchor;
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

void LeggedOdometryManager::replaceRobotPose(const sva::PTransformd & newPose, bool updateVel, bool updateAcc)
{
  stateObservation::kine::Kinematics prevPoseKine =
      conversions::kinematics::fromSva(fbPose_, so::kine::Kinematics::Flags::pose);

  stateObservation::kine::Kinematics newPoseKine =
      conversions::kinematics::fromSva(newPose, so::kine::Kinematics::Flags::pose);

  so::kine::Kinematics deltaKine = newPoseKine * prevPoseKine.getInverse();

  fbPose_ = newPose;
  odometryRobot().posW(fbPose_);
  odometryRobot().forwardKinematics();

  for(auto & contact : maintainedContacts())
  {
    so::kine::Kinematics fbWorldRefKine = prevPoseKine.getInverse() * contact->worldRefKine_;
    so::kine::Kinematics fbWorldRefKineBeforeCorrection =
        prevPoseKine.getInverse() * contact->worldRefKineBeforeCorrection_;

    contact->worldRefKine_ = newPoseKine * fbWorldRefKine;
    contact->worldRefKineBeforeCorrection_ = newPoseKine * fbWorldRefKineBeforeCorrection;

    if(odometryType_ == measurements::OdometryType::Flat)
    {
      contact->worldRefKineBeforeCorrection_.position()(2) = 0.0;
      contact->worldRefKine_.position()(2) = 0.0;
    }
  }

  // we impose the re-computation of the anchor point as the contact references changed.
  k_anchor_ = k_data_ - 1;

  if(updateVel)
  {
    so::Vector3 localLinVel = prevPoseKine.orientation.toMatrix3().transpose() * odometryRobot().velW().linear();
    so::Vector3 localAngVel = prevPoseKine.orientation.toMatrix3().transpose() * odometryRobot().velW().angular();

    sva::MotionVecd vel;

    vel.linear() = newPoseKine.orientation.toMatrix3() * localLinVel;
    vel.angular() = newPoseKine.orientation.toMatrix3() * localAngVel;
    odometryRobot().velW(vel);
    odometryRobot().forwardVelocity();
  }

  if(updateAcc)
  {
    so::Vector3 localLinAcc = prevPoseKine.orientation.toMatrix3().transpose() * odometryRobot().accW().linear();
    so::Vector3 localAngAcc = prevPoseKine.orientation.toMatrix3().transpose() * odometryRobot().accW().angular();

    sva::MotionVecd acc;

    acc.linear() = newPoseKine.orientation.toMatrix3() * localLinAcc;
    acc.angular() = newPoseKine.orientation.toMatrix3() * localAngAcc;
    odometryRobot().accW(acc);
    odometryRobot().forwardAcceleration();
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
  contact.resetLifeTime();

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
    so::kine::Kinematics bodyNewContactKine =
        conversions::kinematics::fromSva(bodyNewContactPoseRobot, so::kine::Kinematics::Flags::pose);

    const sva::PTransformd & worldBodyPoseOdometryRobot =
        odometryRobot().mbc().bodyPosW[odometryRobot().bodyIndexByName(forceSensor.parentBody())];
    so::kine::Kinematics worldBodyKineOdometryRobot =
        conversions::kinematics::fromSva(worldBodyPoseOdometryRobot, so::kine::Kinematics::Flags::pose);

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
  contact.worldRefKineBeforeCorrection_ = contact.worldRefKine_;
  contact.newIncomingWorldRefKine_ = contact.worldRefKine_;

  if(odometryType_ == measurements::OdometryType::Flat) { contact.worldRefKine_.position()(2) = 0.0; }
}

const so::kine::Kinematics & LeggedOdometryManager::recomputeContactKinematics(LoContactWithSensor & contact)
{
  // if the kinematics of the contact in the floating base has not been updated yet (k_est_ = k_iter_ - 1), we cannot
  // use them.
  if(k_data_ != k_iter_)
  {
    BOOST_ASSERT_MSG(false, "This is the first call for the kinematics of the contact in the world for that iteration, "
                            "please use the overload taking the force sensor as a parameter.");
  }
  // if the kinematics of the contact in the floating base have already been updated but the pose of the robot still has
  // not changed, we don't need to recompute the kinematics of the contact in the world.
  if(k_data_ != k_est_)
  {
    const stateObservation::kine::Kinematics worldFbKine =
        conversions::kinematics::fromSva(odometryRobot().posW(), odometryRobot().velW());
    contact.currentWorldKine_ = worldFbKine * contact.contactFbKine_.getInverse();
  };

  return contact.currentWorldKine_;
}

void LeggedOdometryManager::addContactLogEntries(const mc_control::MCController & ctl,
                                                 mc_rtc::Logger & logger,
                                                 const LoContactWithSensor & contact)
{
  const std::string & contactName = contact.name();

  conversions::kinematics::addToLogger(logger, contact.worldRefKine_,
                                       odometryName_ + "_leggedOdometryManager_" + contactName + "_refPose");
  conversions::kinematics::addToLogger(logger, contact.worldFbKineFromRef_,
                                       odometryName_ + "_leggedOdometryManager_" + contactName + "_worldFbKineFromRef");
  conversions::kinematics::addToLogger(logger, contact.currentWorldKine_,
                                       odometryName_ + "_leggedOdometryManager_" + contactName
                                           + "_currentWorldContactKine");
  conversions::kinematics::addToLogger(logger, contact.contactFbKine_,
                                       odometryName_ + "_leggedOdometryManager_" + contactName + "_contactFbKine_");
  conversions::kinematics::addToLogger(logger, contact.worldRefKineBeforeCorrection_,
                                       odometryName_ + "_leggedOdometryManager_" + contactName
                                           + "_refPoseBeforeCorrection");
  conversions::kinematics::addToLogger(logger, contact.newIncomingWorldRefKine_,
                                       odometryName_ + "_leggedOdometryManager_" + contactName
                                           + "_newIncomingWorldRefKine");

  logger.addLogEntry(odometryName_ + "_leggedOdometryManager_" + contactName + "_realRobot_pos", &contact,
                     [&ctl, &contact, this]()
                     { return ctl.realRobot(robotName_).surfacePose(contact.surface()).translation(); });
  logger.addLogEntry(odometryName_ + "_leggedOdometryManager_" + contactName + "_realRobot_ori", &contact,
                     [&ctl, &contact, this]() {
                       return Eigen::Quaterniond(ctl.realRobot(robotName_).surfacePose(contact.surface()).rotation());
                     });

  logger.addLogEntry(odometryName_ + "_leggedOdometryManager_" + contactName + "_isSet", &contact,
                     [&contact]() -> std::string { return contact.isSet() ? "Set" : "notSet"; });

  logger.addLogEntry(odometryName_ + "_leggedOdometryManager_" + contactName + "_lambda", &contact,
                     [&contact]() -> double { return contact.lambda(); });
  logger.addLogEntry(odometryName_ + "_leggedOdometryManager_" + contactName + "_forceNorm", &contact,
                     [&contact]() -> double { return contact.forceNorm(); });
  logger.addLogEntry(odometryName_ + "_leggedOdometryManager_" + contactName + "_lifeTime", &contact,
                     [&contact]() -> double { return contact.lifeTime(); });
  logger.addLogEntry(odometryName_ + "_leggedOdometryManager_" + contactName + "_weightingCoeff", &contact,
                     [&contact]() -> double { return contact.weightingCoeff(); });
}

void LeggedOdometryManager::removeContactLogEntries(mc_rtc::Logger & logger, const LoContactWithSensor & contact)
{
  conversions::kinematics::removeFromLogger(logger, contact.worldRefKine_);
  conversions::kinematics::removeFromLogger(logger, contact.worldRefKineBeforeCorrection_);
  conversions::kinematics::removeFromLogger(logger, contact.worldFbKineFromRef_);
  conversions::kinematics::removeFromLogger(logger, contact.currentWorldKine_);
  conversions::kinematics::removeFromLogger(logger, contact.contactFbKine_);
  conversions::kinematics::removeFromLogger(logger, contact.newIncomingWorldRefKine_);
  logger.removeLogEntries(&contact);
}

void LeggedOdometryManager::correctContactsRef()
{
  for(auto * mContact : maintainedContacts_)
  {
    // we store the pose of the contact before it is corrected
    mContact->worldRefKineBeforeCorrection_ = mContact->worldRefKine_;

    mContact->newIncomingWorldRefKine_ = recomputeContactKinematics(*mContact);

    // double tau = ctl_dt_ / (kappa_ * mContact->lifeTime());
    mContact->weightingCoeff((1 - lambdaInf_) * exp(-kappa_ * mContact->lifeTime()) + lambdaInf_);

    so::kine::Orientation Rtilde(so::Matrix3(mContact->worldRefKineBeforeCorrection_.orientation.toMatrix3().transpose()
                                             * mContact->newIncomingWorldRefKine_.orientation.toMatrix3()));

    so::Vector3 logRtilde =
        so::kine::skewSymmetricToRotationVector(Rtilde.toMatrix3() - Rtilde.toMatrix3().transpose());
    mContact->worldRefKine_.orientation =
        so::Matrix3(mContact->worldRefKineBeforeCorrection_.orientation.toMatrix3()
                    * so::kine::rotationVectorToRotationMatrix(mContact->weightingCoeff() / 2.0 * logRtilde));

    mContact->worldRefKine_.position =
        mContact->worldRefKine_.position()
        + mContact->weightingCoeff()
              * (mContact->newIncomingWorldRefKine_.position() - mContact->worldRefKine_.position());
    if(odometryType_ == measurements::OdometryType::Flat) { mContact->worldRefKine_.position()(2) = 0.0; }
  }

  k_correct_ = k_data_;
}

so::kine::Kinematics LeggedOdometryManager::getContactKineIn(LoContactWithSensor & contact,
                                                             stateObservation::kine::Kinematics & worldTargetKine)
{
  so::kine::Kinematics targetContactKine = worldTargetKine.getInverse() * recomputeContactKinematics(contact);
  return targetContactKine;
}

so::kine::Kinematics LeggedOdometryManager::getAnchorKineIn(stateObservation::kine::Kinematics & worldTargetKine)
{
  if(k_data_ == k_est_) { mc_rtc::log::error_and_throw("Please call initLoop before this function"); }

  so::kine::Kinematics targetAnchorKine;
  targetAnchorKine.position.set().setZero();

  if(worldTargetKine.linVel.isSet()) { targetAnchorKine.linVel.set().setZero(); }

  for(auto * mContact : maintainedContacts_)
  {
    so::kine::Kinematics targetContactKine = getContactKineIn(*mContact, worldTargetKine);
    targetAnchorKine.position() += targetContactKine.position() * mContact->lambda();
    if(targetContactKine.linVel.isSet())
    {
      targetAnchorKine.linVel() += targetContactKine.linVel() * mContact->lambda();
    }
  }

  return targetAnchorKine;
}

stateObservation::Vector3 & LeggedOdometryManager::getCurrentWorldAnchorPos(const mc_control::MCController & ctl,
                                                                            const std::string & bodySensorName)
{
  if(k_data_ == k_est_) { mc_rtc::log::error_and_throw("Please call initLoop before this function"); }

  if(!posUpdatable_)
  {
    mc_rtc::log::warning("No contact detected, the anchor point will be unchanged.");
    return worldAnchorPos_;
  }

  bool linKineUpdatable = false;

  anchorPointMethodChanged_ = false;

  worldAnchorPos_.setZero();

  // checks that the position and orientation can be updated from the currently set contacts and computes the pose of
  // the floating base obtained from each contact
  for(auto * mContact : maintainedContacts_)
  {
    if(!(mContact->isSet() && mContact->wasAlreadySet())) { continue; }
    linKineUpdatable = true;

    const so::kine::Kinematics & worldContactKine = recomputeContactKinematics(*mContact);

    // force weighted sum of the estimated floating base positions
    worldAnchorPos_ += worldContactKine.position() * mContact->lambda();
  }

  if(linKineUpdatable)
  {
    currAnchorFromContacts_ = true;
    if(currAnchorFromContacts_ != prevAnchorFromContacts_) { anchorPointMethodChanged_ = true; }
  }
  else
  {
    // if we cannot update the position (so not the orientations either) using contacts, we use the IMU frame as the
    // anchor frame.
    const auto & imu = ctl.robot(robotName_).bodySensor(bodySensorName);

    const sva::PTransformd & imuXbs = imu.X_b_s();
    so::kine::Kinematics parentImuKine = conversions::kinematics::fromSva(imuXbs, so::kine::Kinematics::Flags::vel);

    const sva::PTransformd & parentPoseW = odometryRobot().bodyPosW(imu.parentBody());

    so::kine::Kinematics worldParentKine =
        conversions::kinematics::fromSva(parentPoseW, so::kine::Kinematics::Flags::vel);

    // pose of the IMU in the world frame
    so::kine::Kinematics worldAnchorKine = worldParentKine * parentImuKine;
    worldAnchorPos_ = worldAnchorKine.position();

    currAnchorFromContacts_ = false;
    if(currAnchorFromContacts_ != prevAnchorFromContacts_) { anchorPointMethodChanged_ = true; }
  }

  return worldAnchorPos_;
}

const so::Vector3 & LeggedOdometryManager::getWorldRefAnchorPos()
{
  // if true, the contacts were not corrected since the last anchor computation, the anchor remains the same.
  bool contactsUnchanged = (k_anchor_ != k_correct_);
  // if true, the anchor has been computed since the beginning of the new iteration
  bool anchorComputed = (k_anchor_ == k_data_);

  // If the anchor point cannot be updated, we return the previously computed value.
  // We also return it if the anchor point has already been computed and the contacts have not been corrected yet (the
  // anchor therefore has not changed yet).
  if(!posUpdatable_ || (anchorComputed && contactsUnchanged)) { return worldRefAnchorPosition_; }

  // "force-weighted" sum of the estimated floating base positions
  worldRefAnchorPosition_.setZero();

  // checks that the position and orientation can be updated from the currently set contacts and computes the pose of
  // the floating base obtained from each contact
  for(auto * mContact : maintainedContacts_)
  {
    const so::kine::Kinematics & worldContactRefKine = mContact->worldRefKine_;

    // force weighted sum of the estimated floating base positions
    worldRefAnchorPosition_ += worldContactRefKine.position() * mContact->lambda();
  }

  k_anchor_ = k_data_;

  return worldRefAnchorPosition_;
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

void LeggedOdometryManager::addToLogger(mc_rtc::Logger & logger, const std::string & leggedOdomCategory)
{
  logger.addLogEntry(leggedOdomCategory + "fbAnchorPos_", [this]() -> so::Vector3 & { return fbAnchorPos_; });
  logger.addLogEntry(leggedOdomCategory + "worldAnchorPos", [this]() { return worldAnchorPos_; });
  logger.addLogEntry(leggedOdomCategory + "odometryRobot_posW",
                     [this]() -> const sva::PTransformd & { return odometryRobot().posW(); });
  logger.addLogEntry(leggedOdomCategory + "odometryRobot_velW",
                     [this]() -> const sva::MotionVecd & { return odometryRobot().velW(); });
  logger.addLogEntry(leggedOdomCategory + "odometryRobot_accW", [this]() { return odometryRobot().accW(); });

  logger.addLogEntry(leggedOdomCategory + "kappa", [this]() { return kappa_; });
  logger.addLogEntry(leggedOdomCategory + "lambdaInf", [this]() { return lambdaInf_; });

  logger.addLogEntry(leggedOdomCategory + "OdometryType",
                     [this]() -> std::string { return measurements::odometryTypeToSstring(odometryType_); });
}

} // namespace mc_state_observation::odometry
