/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/SurfaceTransformTask.h>

#include <mc_rtc/gui/Transform.h>

namespace mc_tasks
{

SurfaceTransformTask::SurfaceTransformTask(const std::string & surfaceName,
                                           const mc_rbdyn::Robots & robots,
                                           unsigned int robotIndex,
                                           double stiffness,
                                           double weight)
: TrajectoryTaskGeneric<tasks::qp::SurfaceTransformTask>(robots, robotIndex, stiffness, weight),
  surfaceName(surfaceName)
{
  const mc_rbdyn::Robot & robot = robots.robot(rIndex);
  if(!robot.hasSurface(surfaceName))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_tasks::SurfaceTransformTask] No surface named {} in {}",
                                                     surfaceName, robot.name());
  }
  std::string bodyName = robot.surface(surfaceName).bodyName();
  sva::PTransformd curPos = robot.surface(surfaceName).X_0_s(robot);
  finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, curPos, robot.surface(surfaceName).X_b_s());

  type_ = "surfaceTransform";
  name_ = "surface_transform_" + robot.name() + "_" + surfaceName;
}

void SurfaceTransformTask::reset()
{
  TrajectoryTaskGeneric::reset();
  const auto & robot = robots.robot(rIndex);
  sva::PTransformd curPos = robot.surfacePose(surfaceName);
  errorT->target(curPos);
}

/*! \brief Load parameters from a Configuration object */
void SurfaceTransformTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{

  // Current surface position
  sva::PTransformd X_0_t = surfacePose();

  // Apply global world transformations first
  if(config.has("targetSurface"))
  {
    const auto & c = config("targetSurface");
    const auto robotIndex = robotIndexFromConfig(c, solver.robots(), name());
    targetSurface(robotIndex, c("surface"),
                  {c("offset_rotation", Eigen::Matrix3d::Identity().eval()),
                   c("offset_translation", Eigen::Vector3d::Zero().eval())});
    X_0_t = this->target();
  }
  else if(config.has("target"))
  {
    X_0_t = config("target");
  }
  else if(config.has("relative"))
  {
    const auto & robot = robotFromConfig(config("relative"), solver.robots(), name());
    std::string s1 = config("relative")("s1");
    std::string s2 = config("relative")("s2");
    sva::PTransformd target = config("relative")("target");
    auto X_0_s1 = robot.surfacePose(s1);
    auto X_0_s2 = robot.surfacePose(s2);
    auto X_s1_s2 = X_0_s2 * X_0_s1.inv();
    X_s1_s2.translation() = X_s1_s2.translation() / 2;
    auto X_0_relative = X_s1_s2 * X_0_s1;
    this->target(target * X_0_relative);
  }
  else
  {
    if(config.has("targetPosition"))
    {
      X_0_t.translation() = config("targetPosition");
    }
    if(config.has("targetRotation"))
    {
      X_0_t.rotation() = config("targetRotation");
    }
  }

  if(config.has("moveWorld"))
  {
    sva::PTransformd move = config("moveWorld");
    X_0_t = X_0_t * move;
  }

  if(config.has("move"))
  {
    sva::PTransformd move = config("move");
    X_0_t = move * X_0_t;
  }

  if(config.has("overwriteRPY"))
  {
    // Only modify the specified DoF of the rotation
    mc_rtc::overwriteRotationRPY(config("overwriteRPY"), X_0_t.rotation());
  }

  this->target(X_0_t);

  TrajectoryBase::load(solver, config);
}

sva::PTransformd SurfaceTransformTask::target() const
{
  return errorT->target();
}

const sva::PTransformd SurfaceTransformTask::surfacePose() const
{
  return robots.robot(rIndex).surfacePose(surfaceName);
}

void SurfaceTransformTask::target(const sva::PTransformd & pose)
{
  errorT->target(pose);
}

void SurfaceTransformTask::targetSurface(unsigned int robotIndex,
                                         const std::string & surfaceName,
                                         const sva::PTransformd & offset)
{
  const auto & robot = robots.robot(robotIndex);
  sva::PTransformd targetSurface = robot.surface(surfaceName).X_0_s(robot);
  sva::PTransformd targetPos = offset * targetSurface;
  target(targetPos);
}

void SurfaceTransformTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_surface_pose", this, [this]() {
    const auto & robot = robots.robot(rIndex);
    return robot.surface(surfaceName).X_0_s(robot);
  });
  logger.addLogEntry(name_ + "_target_pose", this, [this]() { return target(); });
}

std::function<bool(const mc_tasks::MetaTask &, std::string &)> SurfaceTransformTask::buildCompletionCriteria(
    double dt,
    const mc_rtc::Configuration & config) const
{
  if(config.has("wrench"))
  {
    if(!robots.robot(rIndex).surfaceHasIndirectForceSensor(surfaceName))
    {
      mc_rtc::log::error_and_throw<std::invalid_argument>("[{}] Attempted to use \"wrench\" as completion criteria but "
                                                          "surface \"{}\" is not attached to a force sensor",
                                                          name(), surfaceName);
    }
    sva::ForceVecd target_w = config("wrench");
    Eigen::Vector6d target = target_w.vector();
    Eigen::Vector6d dof = Eigen::Vector6d::Ones();
    for(int i = 0; i < 6; ++i)
    {
      if(std::isnan(target(i)))
      {
        dof(i) = 0.;
        target(i) = 0.;
      }
      else if(target(i) < 0)
      {
        dof(i) = -1.;
      }
    }
    return [dof, target](const mc_tasks::MetaTask & t, std::string & out) {
      const auto & self = static_cast<const mc_tasks::SurfaceTransformTask &>(t);
      Eigen::Vector6d w = self.robots.robot(self.rIndex).surfaceWrench(self.surface()).vector();
      for(int i = 0; i < 6; ++i)
      {
        if(dof(i) * fabs(w(i)) < target(i))
        {
          return false;
        }
      }
      out += "wrench";
      return true;
    };
  }
  return MetaTask::buildCompletionCriteria(dt, config);
}

void SurfaceTransformTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTaskGeneric<tasks::qp::SurfaceTransformTask>::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Transform(
                     "pos_target", [this]() { return this->target(); },
                     [this](const sva::PTransformd & pos) { this->target(pos); }),
                 mc_rtc::gui::Transform("pos", [this]() {
                   return robots.robot(rIndex).surface(surfaceName).X_0_s(robots.robot(rIndex));
                 }));
}

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "surfaceTransform",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      const auto robotIndex = robotIndexFromConfig(config, solver.robots(), "surfaceTransform");
      auto t = std::make_shared<mc_tasks::SurfaceTransformTask>(config("surface"), solver.robots(), robotIndex);
      t->load(solver, config);
      return t;
    });
}
