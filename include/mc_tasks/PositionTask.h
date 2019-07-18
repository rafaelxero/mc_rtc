/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

namespace mc_tasks
{

/*! \brief Control the position of a body

 * This task is thin wrapper around the appropriate tasks in Tasks.
 *
 */
struct MC_TASKS_DLLAPI PositionTask : public TrajectoryTaskGeneric<tasks::qp::PositionTask>
{
public:
  friend struct EndEffectorTask;

  /*! \brief Constructor
   *
   * \param bodyName Name of the body to control
   *
   * \param robots Robots controlled by this task
   *
   * \param robotIndex Index of the robot controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  PositionTask(const std::string & bodyName,
               const mc_rbdyn::Robots & robots,
               unsigned int robotIndex,
               double stiffness = 2.0,
               double weight = 500);

  /*! \brief Constructor
   *
   * @see PositionTask
   *
   * \param bodyPoint Point on the body being controlled, in body coordinates
   *
   */
  PositionTask(const std::string & bodyName,
               const Eigen::Vector3d & bodyPoint,
               const mc_rbdyn::Robots & robots,
               unsigned int robotIndex,
               double stiffness = 2.0,
               double weight = 500);

  virtual ~PositionTask() = default;

  /*! \brief Reset the task
   *
   * Set the task objective to the current body position
   */
  virtual void reset() override;

  /*! \brief Get the body position target */
  Eigen::Vector3d position();

  /*! \brief Set the body position target
   *
   * \param pos Body position in world frame
   *
   */
  void position(const Eigen::Vector3d & pos);

  /*! \brief Change the body position target by a given amount
   *
   * \param com Modification applied to the body position
   *
   */
  void move_position(const Eigen::Vector3d & pos);
  
  /*! \brief Get the body point being controlled
   */
  Eigen::Vector3d bodyPoint() const;

  /*! \brief Set the body point being controlled
   *
   * \param bodyPoint point position in body frame
   *
   */
  void bodyPoint(const Eigen::Vector3d & bodyPoint);

  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;

protected:
  std::string bodyName;
  unsigned int bIndex;
  virtual void addToLogger(mc_rtc::Logger & logger) override;
  virtual void removeFromLogger(mc_rtc::Logger & logger) override;
};

} // namespace mc_tasks
