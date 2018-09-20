#include <mc_tasks/WrenchTask.h>

namespace mc_tasks
{

  WrenchTask::WrenchTask(const std::string & bodyName, const mc_rbdyn::Robots & robots,
                         unsigned int robotIndex, double weight) :
    WrenchTask(bodyName, Eigen::Vector3d::Zero(), robots, robotIndex, weight)
  {
  }
  
  WrenchTask::WrenchTask(const std::string & bodyName, const Eigen::Vector3d& bodyPoint,
                         const mc_rbdyn::Robots & robots, unsigned int robotIndex,
                         double weight) :
    tasks::qp::WrenchTask(robots.mbs(), robotIndex, weight),
    bodyName(bodyName), bIndex(0)
  {
    const mc_rbdyn::Robot & robot = robots.robot(rIndex);
    bIndex = robot.bodyIndexByName(bodyName);
  }

  void WrenchTask::reset()
  {
  }

  Eigen::Vector6d WrenchTask::wrench()
  {
  }

  void WrenchTask::wrench(const Eigen::Vector6d & wrench)
  {
  }

  Eigen::Vector3d WrenchTask::bodyPoint() const
  {
  }
}
