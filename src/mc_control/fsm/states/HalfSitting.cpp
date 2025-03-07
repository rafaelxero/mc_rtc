/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/HalfSitting.h>

namespace mc_control
{

namespace fsm
{

void HalfSittingState::start(Controller & ctl)
{
  robot_ = config_("robot", ctl.robot().name());
  auto postureTask = ctl.getPostureTask(robot_);
  default_stiffness_ = postureTask->stiffness();
  postureTask->stiffness(config_("stiffness", static_cast<const double &>(default_stiffness_)));
  /* Set the halfSitPose in posture Task */
  const auto & halfSit = ctl.robot().module().stance();
  const auto & ref_joint_order = ctl.robot().refJointOrder();
  auto posture = postureTask->posture();
  for(unsigned int i = 0; i < ref_joint_order.size(); ++i)
  {
    if(ctl.robot().hasJoint(ref_joint_order[i]))
    {
      posture[ctl.robot().jointIndexByName(ref_joint_order[i])] = halfSit.at(ref_joint_order[i]);
    }
  }
  postureTask->posture(posture);
  /** If eval is not provided we want the exit condition to be immediately valid */
  eval_threshold_ = config_("eval", std::numeric_limits<double>::infinity());
}

bool HalfSittingState::run(Controller & ctl)
{
  auto postureTask = ctl.getPostureTask(robot_);
  if(postureTask->eval().norm() < eval_threshold_)
  {
    postureTask->stiffness(default_stiffness_);
    output("OK");
    return true;
  }
  return false;
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("HalfSitting", mc_control::fsm::HalfSittingState)
