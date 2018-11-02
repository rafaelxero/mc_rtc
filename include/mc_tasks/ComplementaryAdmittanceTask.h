#pragma once

namespace mc_tasks
{

struct MC_TASKS_DLLAPI ComplementaryAdmittanceTask
{
public:

  ComplementaryAdmittanceTask(const std::string & bodyName, const mc_rbdyn::Robots & robots,
                              unsigned int robotIndex, double gainP, double gainD, double weight = 500);

  void reset();

  sva::ForceVecd measuredWrench() const
  {
    // sva::ForceVecd w_fsactual = sensor_.removeGravity(robot_);
    // return X_fsactual_surf_.dualMul(w_fsactual);
  }
}
