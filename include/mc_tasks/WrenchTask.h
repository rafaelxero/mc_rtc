#pragma once

namespace mc_tasks
{

  /*! \brief Drives the calculated wrench as close as possible to the desired one
    
   * This task is thin wrapper around the appropriate task in Tasks.
   *
   */
  struct MC_TASKS_DLLAPI WrenchTask
  {
  public:

    WrenchTask(const std::string & bodyName, const mc_rbdyn::Robots & robots,
               unsigned int robotIndex, double weight = 500);

    WrenchTask(const std::string & bodyName, const Eigen::Vector3d& bodyPoint,
               const mc_rbdyn::Robots & robots, unsigned int robotIndex,
               double weight = 500);

    virtual void reset() override;

    Eigen::Vector6d wrench();

    void wrench(const Eigen::Vector6d & wrench);

    Eigen::Vector3d bodyPoint() const;

    void bodyPoint(const Eigen::Vector6d & bodyPoint);

  protected:

    std::string bodyName;
    unsigned int bIndex;
  };

}
