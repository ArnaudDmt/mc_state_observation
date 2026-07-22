/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/logging.h>
#include <mc_state_observation/MOCAPVisualizer.h>
#include <mc_state_observation/gui_helpers.h>

#include <RBDyn/CoM.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <iostream>

namespace mc_state_observation
{
MOCAPVisualizer::MOCAPVisualizer(const std::string & type, double dt) : mc_observers::Observer(type, dt) {}

///////////////////////////////////////////////////////////////////////
/// --------------------------Core functions---------------------------
///////////////////////////////////////////////////////////////////////

void MOCAPVisualizer::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  std::string numExpe;
  numExpe = static_cast<std::string>(config("numExpe"));
  csvPath_ = "/home/arnaud/Documents/KineticsObserver/logs/expHRP5/" + numExpe
             + "/MoCap/c++Reader/resultingData/aligned_data.csv";
}

void MOCAPVisualizer::reset(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);

  my_robots_ = mc_rbdyn::Robots::make();
  my_robots_->robotCopy(robot, robot.name());
  ctl.gui()->addElement(
      {"Robots"},
      mc_rtc::gui::Robot("MOCAPVisualizer", [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));
  /*
ctl.gui()->addElement({"Robots"},
                    mc_rtc::gui::Robot("Real", [&ctl]() -> const mc_rbdyn::Robot & { return ctl.realRobot(); }));*/

  // MOCAP DATA TEST
  tempMocapData_.kine.setZero(so::kine::Kinematics::Flags::position | so::kine::Kinematics::Flags::orientation);
  extractMocapData();

  X_0_fb_ = realRobot.posW(); // we initialize the mocap robot with the real robot
  X_0_fb_init_ = realRobot.posW(); // we initialize the mocap robot with the real robot

  for(auto forceSensor : realRobot.forceSensors()) { contacts_.insert(std::make_pair(forceSensor.name(), Contact())); }
  updateContacts(ctl);
  for(auto forceSensor : realRobot.forceSensors())
  {
    addContactsLogs(forceSensor.name(), const_cast<mc_control::MCController &>(ctl).logger());
  }
}

bool MOCAPVisualizer::run(const mc_control::MCController & ctl)
{
  // std::cout << "\033[1;31m" << std::endl << "New iteration: " << std::endl << "\033[0m\n";
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);
  auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();

  std::string mocapTime = std::to_string(currentMocapDataTime_);
  if(mocapTime[mocapTime.find_last_not_of('0')] == '.')
  {
    mocapTime.erase(mocapTime.find_last_not_of('0') + 2, std::string::npos);
  }
  else { mocapTime.erase(mocapTime.find_last_not_of('0') + 1, std::string::npos); }
  if(mocapTime.back() == '.') { mocapTime.pop_back(); }

  if(!mocapFinished)
  {
    if(mocapDataTable_.find(mocapTime) != mocapDataTable_.end()) { tempMocapData_ = mocapDataTable_.at(mocapTime); }
    else
    {
      mocapFinished = true;
      logger.removeLogEntry(category_ + "_MOCAP_pos");
      logger.removeLogEntry(category_ + "_MOCAP_ori");
    }
  }
  currentMocapDataTime_ += ctl.timeStep;

  // tempMocapData_ is an increment in position and orientation on the initial pose

  so::kine::Kinematics newKine; // we initialize the new Kinematics with the initial ones
  newKine.position = X_0_fb_init_.translation();
  newKine.orientation = so::Matrix3(X_0_fb_init_.rotation().transpose());

  newKine = newKine * tempMocapData_.kine;
  /*
  so::Vector3 newPos = X_0_fb_init_.translation() + tempMocapData_.kine.position();
  so::Matrix3 newOri =
      (X_0_fb_init_.rotation().transpose() * tempMocapData_.kine.orientation.toMatrix3()).transpose();
  */

  X_0_fb_.translation() = newKine.position();
  X_0_fb_.rotation() = newKine.orientation.toMatrix3().transpose();

  my_robots_->robot().mbc().q = ctl.realRobot().mbc().q;
  update(my_robots_->robot());

  updateContacts(ctl);
}

///////////////////////////////////////////////////////////////////////
/// -------------------------Called functions--------------------------
///////////////////////////////////////////////////////////////////////

void MOCAPVisualizer::update(mc_control::MCController & ctl) // this function is called by the pipeline if the
                                                             // update is set to true in the configuration file
{
  auto & realRobot = ctl.realRobot(robot_);
  update(realRobot);
}

void MOCAPVisualizer::update(mc_rbdyn::Robot & robot)
{
  robot.posW(X_0_fb_);
  // robot.velW(v_fb_0_.vector());
}

void MOCAPVisualizer::updateContacts(const mc_control::MCController & ctl)
{
  const auto & realRobot = ctl.realRobot(robot_);
  const auto & mocapRobot = my_robots_->robot();

  for(auto contact : contacts_)
  {
    Contact & ct = contact.second;
    const std::string & fsName = contact.first;
    const mc_rbdyn::ForceSensor & forceSensor = realRobot.forceSensor(fsName);
    if(fsName.find("LeftHand") == std::string::npos)
    {

      so::kine::Kinematics worldContactKineRef;
      worldContactKineRef.setZero(so::kine::Kinematics::Flags::position);

      // getting the position in the world of the new contact
      const sva::PTransformd & bodyNewContactPoseRobot = forceSensor.X_p_f();
      so::kine::Kinematics bodyNewContactKine;
      bodyNewContactKine.setZero(so::kine::Kinematics::Flags::all);
      bodyNewContactKine.position = bodyNewContactPoseRobot.translation();
      bodyNewContactKine.orientation = so::Matrix3(bodyNewContactPoseRobot.rotation().transpose());

      so::kine::Kinematics worldBodyKineOdometryRobot;

      worldBodyKineOdometryRobot.position =
          mocapRobot.mbc().bodyPosW[mocapRobot.bodyIndexByName(forceSensor.parentBody())].translation();
      worldBodyKineOdometryRobot.orientation = so::Matrix3(
          mocapRobot.mbc().bodyPosW[mocapRobot.bodyIndexByName(forceSensor.parentBody())].rotation().transpose());
      worldBodyKineOdometryRobot.linVel =
          mocapRobot.mbc().bodyVelW[mocapRobot.bodyIndexByName(forceSensor.parentBody())].linear();
      worldBodyKineOdometryRobot.angVel =
          mocapRobot.mbc().bodyVelW[mocapRobot.bodyIndexByName(forceSensor.parentBody())].angular();
      worldBodyKineOdometryRobot.linAcc =
          worldBodyKineOdometryRobot.orientation.toMatrix3()
          * mocapRobot.mbc().bodyAccB[mocapRobot.bodyIndexByName(forceSensor.parentBody())].linear();
      worldBodyKineOdometryRobot.angAcc =
          worldBodyKineOdometryRobot.orientation.toMatrix3()
          * mocapRobot.mbc().bodyAccB[mocapRobot.bodyIndexByName(forceSensor.parentBody())].angular();

      // std::cout << std::endl << "bodyNewContactKine" << std::endl << bodyNewContactKine << std::endl;
      // std::cout << std::endl << "worldBodyKineOdometryRobot" << std::endl << worldBodyKineOdometryRobot << std::endl;
      so::kine::Kinematics worldNewContactKineOdometryRobot = worldBodyKineOdometryRobot * bodyNewContactKine;

      contacts_.at(forceSensor.name()).worldRefKine_.position = worldNewContactKineOdometryRobot.position();
      contacts_.at(forceSensor.name()).worldRefKine_.orientation = worldNewContactKineOdometryRobot.orientation;
      contacts_.at(forceSensor.name()).quat_ = worldNewContactKineOdometryRobot.orientation.toQuaternion().inverse();
    }
    else
    {
      sva::PTransformd surfaceKine = mocapRobot.surfacePose("LeftHandCloseContact");
      contacts_.at(forceSensor.name()).worldRefKine_.position = surfaceKine.translation();
      contacts_.at(forceSensor.name()).worldRefKine_.orientation = so::Matrix3(surfaceKine.rotation().transpose());
      contacts_.at(forceSensor.name()).quat_ =
          contacts_.at(forceSensor.name()).worldRefKine_.orientation.toQuaternion().inverse();
    }
  }
}
///////////////////////////////////////////////////////////////////////
/// -------------------------------Logs--------------------------------
///////////////////////////////////////////////////////////////////////

void MOCAPVisualizer::addToLogger(const mc_control::MCController &,
                                  mc_rtc::Logger & logger,
                                  const std::string & category)
{
  logger.addLogEntry(category + "_MOCAP_pos",
                     [this]() -> const so::Vector3 & { return tempMocapData_.kine.position(); });
  logger.addLogEntry(category + "_MOCAP_ori",
                     [this]() -> so::Quaternion { return tempMocapData_.kine.orientation.toQuaternion().inverse(); });
  logger.addLogEntry(category + "_mocap_fb_posW", [this]() -> const sva::PTransformd & { return X_0_fb_; });
  logger.addLogEntry(category + "_mocap_fb_yaw",
                     [this]() -> const double
                     { return -so::kine::rotationMatrixToYawAxisAgnostic(X_0_fb_.rotation()); });
}

void MOCAPVisualizer::addContactsLogs(const std::string & name, mc_rtc::Logger & logger)
{
  logger.addLogEntry("MOCAPVisualizer_contacts_" + name + "_position",
                     [this, name]() -> so::Vector3 { return contacts_.at(name).worldRefKine_.position(); });
  logger.addLogEntry("MOCAPVisualizer_contacts_" + name + "_orientation",
                     [this, name]() -> so::Quaternion { return contacts_.at(name).quat_; });
  logger.addLogEntry("MOCAPVisualizer_contacts_" + name + "_orientation_RollPitchYaw",
                     [this, name]() -> so::Vector3
                     { return contacts_.at(name).worldRefKine_.orientation.toRollPitchYaw(); });
}

void MOCAPVisualizer::removeFromLogger(mc_rtc::Logger & logger, const std::string & category) {}

void MOCAPVisualizer::addToGUI(const mc_control::MCController &,
                               mc_rtc::gui::StateBuilder & gui,
                               const std::vector<std::string> & category)
{
  using namespace mc_rtc::gui;
  // clang-format off

  // clang-format on
}

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

void MOCAPVisualizer::extractMocapData()
{
  std::string fname;
  // cout << "Enter the file name: ";
  //  cin >> fname;
  fname = csvPath_;
  std::ofstream myfile;
  myfile.open("/home/arnaud/Documents/KineticObserver/logs/expHRP5/MoCap/c++Reader/example.txt");
  std::vector<std::vector<std::string>> content;
  std::vector<std::string> row;

  std::vector<std::vector<std::string>> newFile;
  // vector<string> newRow(8);
  std::string line, word;

  std::fstream file(fname, std::ios::in);

  if(file.is_open())
  {
    int i = 0;
    so::kine::Kinematics currentKine;
    currentKine.setZero(so::kine::Kinematics::Flags::position | so::kine::Kinematics::Flags::orientation);
    so::kine::Kinematics initKine;
    initKine.setZero(so::kine::Kinematics::Flags::position | so::kine::Kinematics::Flags::orientation);
    so::kine::Kinematics realignOrientation;
    realignOrientation.setZero(so::kine::Kinematics::Flags::position);
    so::Matrix3 realignmentMatrix = so::Matrix3::Zero();

    // realignmentMatrix(0, 0) = 1.0; // realignmentMatrix(0, 0) = 1.0; // realignmentMatrix(0, 1) = -1.0;
    // realignmentMatrix(1, 1) = 1.0; // realignmentMatrix(1, 1) = 1.0; // realignmentMatrix(1, 0) = 1.0;
    // realignmentMatrix(2, 2) = 1.0;
    // realignOrientation.orientation = realignmentMatrix;
    so::Matrix3 realignMat;
    realignMat << 0, 0, 1, 1, 0, 0, 0, 1, 0;
    realignOrientation.orientation =
        so::Matrix3(realignMat); // so::kine::rotationVectorToRotationMatrix(so::Vector3(0, 0, 0)
                                 //  * so::kine::rotationVectorToRotationMatrix(so::Vector3(0, 0, 0)));
    bool intitialized = false;
    while(getline(file, line))
    {
      row.clear();
      std::vector<std::string> newRow(8);
      std::stringstream str(line);

      while(getline(str, word, ',')) row.push_back(word);
      i++;
      if(i > 1)
      {
        so::Vector4 quat;
        if(mocapDataTable_.find(row.at(1)) != mocapDataTable_.end()) // if timecode already exists we compute the mean
        {
          /*          auto & alreadyExistingMocapData = mocapDataTable_.at(row.at(1));

          alreadyExistingMocapData.time = (alreadyExistingMocapData.time + std::stod(row.at(1))) / 2;
          quat(0) = (alreadyExistingMocapData.ori.toQuaternion().w() + std::stod(row.at(5))) / 2;
          quat(1) = (alreadyExistingMocapData.ori.toQuaternion().x() + std::stod(row.at(2))) / 2;
          quat(2) = (alreadyExistingMocapData.ori.toQuaternion().y() + std::stod(row.at(3))) / 2;
          quat(3) = (alreadyExistingMocapData.ori.toQuaternion().z() + std::stod(row.at(4))) / 2;
          tempMocapData_.ori.fromVector4(quat);
          alreadyExistingMocapData.pos(0) = (alreadyExistingMocapData.pos(0) + std::stod(row.at(6))) / 2;
          alreadyExistingMocapData.pos(1) = (alreadyExistingMocapData.pos(1) + std::stod(row.at(7))) / 2;
          alreadyExistingMocapData.pos(2) = (alreadyExistingMocapData.pos(2) + std::stod(row.at(8))) / 2;
          */
        }
        else // new row
        {
          tempMocapData_.indexReader = std::stoi(row.at(0));
          tempMocapData_.time = std::stoi(row.at(1));
          quat(0) = std::stod(row.at(5)); // w
          quat(1) = std::stod(row.at(2)); // x
          quat(2) = std::stod(row.at(3)); // y
          quat(3) = std::stod(row.at(4)); // z

          currentKine.orientation = Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3)).normalized();

          so::kine::Orientation oriTransfo( // so::Quaternion(Eigen::AngleAxis(-M_PI_2, Eigen::Vector3d::UnitZ()))
                                            //*
              so::Quaternion(quat(0), quat(1), quat(2), quat(3)));

          {

            so::Matrix3 tempOri;
            tempOri << 1, 0, 0, 0, 0, -1, 0, 1, 0; // 0, 0, -1, 0, 1, 0, 1, 0, 0;
            so::kine::Orientation fixOrientations(tempOri);
            oriTransfo = so::Matrix3(
                so::Quaternion(Eigen::AngleAxis(0.0269199223, Eigen::Vector3d::UnitY()))
                    .toRotationMatrix() // 0.0269199223 comes from the compensation of the very slight disalignment
                                        // between the world and the mocap axis causing the mocap to go down over the
                                        // time
                * tempOri * so::Quaternion(Eigen::AngleAxis(M_PI_2, Eigen::Vector3d::UnitY())).toRotationMatrix()
                * oriTransfo.toMatrix3() * tempOri.transpose());

            /*
             oriTransfo.toQuaternion();
             oriTransfo.getMatrixRefUnsafe().reset();
             oriTransfo.getQuaternionRefUnsafe().coeffs().head<3>() =
                 tempOri * oriTransfo.toQuaternion().coeffs().head<3>();
                 */
          }

          currentKine.orientation = oriTransfo;

          currentKine.position()(0) = std::stod(row.at(6));
          currentKine.position()(1) = std::stod(row.at(7));
          currentKine.position()(2) = std::stod(row.at(8));

          currentKine.position = realignOrientation.orientation * currentKine.position();
          // currentKine = realignOrientation * currentKine;

          if(!intitialized)
          {
            initKine = currentKine;
            tempMocapData_.kine.position().setZero();
            tempMocapData_.kine.orientation.setZeroRotation();

            intitialized = true;
          }

          tempMocapData_.kine = initKine.getInverse() * currentKine;
          mocapDataTable_.insert(std::make_pair(row.at(1), tempMocapData_));

          newRow.at(0) = row.at(1);
          newRow.at(1) = row.at(2);
          newRow.at(2) = row.at(3);
          newRow.at(3) = row.at(4);
          newRow.at(4) = row.at(5);
          newRow.at(5) = row.at(6);
          newRow.at(6) = row.at(7);
          newRow.at(7) = row.at(8);

          newFile.push_back(newRow);

          for(int j = 0; j < newRow.size(); j++) { myfile << newRow[j] << " "; }
          myfile << std::endl;
        }
      }
    }
    myfile.close();
  }
  else
    mc_rtc::log::error_and_throw<std::runtime_error>("Could not open the file\n");
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("MOCAPVisualizer", mc_state_observation::MOCAPVisualizer)
