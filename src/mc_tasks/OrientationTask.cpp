/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/OrientationTask.h>

#include <mc_rtc/gui/Rotation.h>

namespace mc_tasks
{

OrientationTask::OrientationTask(const std::string & bodyName,
                                 const mc_rbdyn::Robots & robots,
                                 unsigned int robotIndex,
                                 double stiffness,
                                 double weight)
: TrajectoryTaskGeneric<tasks::qp::OrientationTask>(robots, robotIndex, stiffness, weight), bodyName(bodyName),
  bIndex(0)
{
  if(robotIndex >= robots.size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_tasks::OrientationTask] No robot with index {}, robots.size() {}", robotIndex, robots.size());
  }
  const auto & robot = robots.robot(robotIndex);
  if(!robot.hasBody(bodyName))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_tasks::OrientationTask] No body named {} in {}", bodyName,
                                                     robot.name());
  }
  bIndex = robot.bodyIndexByName(bodyName);

  Eigen::Matrix3d curOri = robot.mbc().bodyPosW[bIndex].rotation();
  finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, curOri);
  type_ = "orientation";
  name_ = "orientation_" + robot.name() + "_" + bodyName;
}

void OrientationTask::reset()
{
  TrajectoryTaskGeneric::reset();
  const auto & robot = robots.robot(rIndex);
  auto curOri = robot.mbc().bodyPosW[bIndex].rotation();
  errorT->orientation(curOri);
}

void OrientationTask::orientation(const Eigen::Matrix3d & ori)
{
  errorT->orientation(ori);
}

Eigen::Matrix3d OrientationTask::orientation()
{
  return errorT->orientation();
}

void OrientationTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTaskGeneric<tasks::qp::OrientationTask>::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Rotation(
                     "ori_target",
                     [this]() -> sva::PTransformd {
                       const auto & curPos = robots.robot(rIndex).mbc().bodyPosW[bIndex];
                       return sva::PTransformd(this->orientation(), curPos.translation());
                     },
                     [this](const Eigen::Quaterniond & ori) { this->orientation(ori.toRotationMatrix()); }),
                 mc_rtc::gui::Rotation("ori", [this]() -> sva::PTransformd {
                   const auto & curPos = robots.robot(rIndex).mbc().bodyPosW[bIndex];
                   return curPos;
                 }));
}

void OrientationTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_target", this, [this]() { return Eigen::Quaterniond(orientation()); });
  logger.addLogEntry(name_, this,
                     [this]() { return Eigen::Quaterniond(robots.robot(rIndex).mbc().bodyPosW[bIndex].rotation()); });
}

} // namespace mc_tasks
